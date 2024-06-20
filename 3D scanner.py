# -*- coding: utf-8 -*-
"""
Created on Thu Jan 21 16:38:24 2021

@author: wouter
"""
# https://stackoverflow.com/questions/7546050/switch-between-two-frames-in-tkinter
# https://stackoverflow.com/questions/14817210/using-buttons-in-tkinter-to-navigate-to-different-pages-of-the-application

# pyinstaller.exe --onefile --windowed --icon="E:/Documenten/3D-scanner/Arduino/icoontje3dscan_WDJ_icon.ico" E:/Documenten/3D-scanner/Arduino/DraaischijfMainV4.py
import sys
sys.path.append(r'C:\Users\elibe\OneDrive\Documentos\Prueba_3D_app\3D scanner\3D scanner\Main program\build\Debug')
import prueba_1
import serial 
import keyboard #pip install keyboard
import pyrealsense2 as rs 
import open3d as o3d #pip install open3d
import numpy as np #pip install numpy
import time
from tkinter import font as tkfont
import tkinter.filedialog
import tkinter as tk
from PIL import ImageTk
from PIL import Image
import ast
from tkinter.ttk import Progressbar

class Arduino():
    
    def __init__(self,comPort,baudRate,timeout):
        self.comPort = comPort
        self.baudRate = baudRate
        self.timeout = timeout
        self.totalAngle = 0
        self.s = serial.Serial(self.comPort,self.baudRate,timeout = self.timeout)
        time.sleep(2)
        self.currentstep = 0
       
    def rotate(self,steps):
        self.steps = steps
        self.negative = int(self.steps < 0)
        if self.negative:
            self.steps = -self.steps
        self.stepsAsString = str(self.steps)
        self.s.write([self.negative])
        for i in range(5-len(self.stepsAsString)):
            self.s.write([0]) 
        for i in range(len(self.stepsAsString)):
            self.s.write([int(self.stepsAsString[i])])
            
    def waitForRotation(self):
        while True:
            self.data = str(self.s.readline()) 
            if self.data != "b''":
                self.currentstep += self.steps 
                if int(str(self.data)[2:len(self.data)-1]) != self.steps:
                    self.close()
                    raise Exception("The Arduino returned the wrong number of steps!")
                break
                    
    def  giveAngle(self):
        self.gearRatio = 6
        self.currentAngle = ((self.currentstep * 360)/2048)/self.gearRatio 
        return self.currentAngle
        
    def close(self):
        self.s.close()
    

class Scan():
    def __init__(self, width, height, framerate, autoexposureFrames, backDistance):
        self.width = width
        self.height = height
        self.framerate = framerate
        self.backDistance = backDistance
        self.autoexposureFrames = autoexposureFrames
        self.main_pcd = o3d.geometry.PointCloud()
        
        self.pipe = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.any, self.framerate)
        self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.any, self.framerate)
        
        # Post-processing filters
        self.depth_to_disparity = rs.disparity_transform(True)
        self.disparity_to_depth = rs.disparity_transform(False)
        self.dec_filter = rs.decimation_filter()
        self.temp_filter = rs.temporal_filter()
        self.spat_filter = rs.spatial_filter()
        self.hole_filter = rs.hole_filling_filter()
        self.threshold = rs.threshold_filter(0.17, 0.4)
        
        self.dtr = np.pi / 180
        self.distance = 0.425 - self.backDistance
        self.bbox = o3d.geometry.AxisAlignedBoundingBox((-0.13, -0.13, 0), (0.13, 0.13, 0.2))
        
    def startPipeline(self):
        self.pipe.start(self.config)
        self.align = rs.align(rs.stream.color)
        print("Pipeline started")
    
    def stopPipeline(self):
        self.pipe.stop()
        self.pipe = None
        self.config = None
        print("Pipeline stopped")
        
    def takeFoto(self):
        print("Photo taken successfully!")
        for i in range(self.autoexposureFrames):
            self.frameset = self.pipe.wait_for_frames()
        
        self.frameset = self.pipe.wait_for_frames()
        self.frameset = self.align.process(self.frameset)
        self.profile = self.frameset.get_profile()
        self.depth_intrinsics = self.profile.as_video_stream_profile().get_intrinsics()
        self.w, self.h = self.depth_intrinsics.width, self.depth_intrinsics.height
        self.fx, self.fy = self.depth_intrinsics.fx, self.depth_intrinsics.fy
        self.px, self.py = self.depth_intrinsics.ppx, self.depth_intrinsics.ppy
        
        self.color_frame = self.frameset.get_color_frame()
        self.depth_frame = self.frameset.get_depth_frame()
        
        self.intrinsic = o3d.camera.PinholeCameraIntrinsic(self.w, self.h, self.fx, self.fy, self.px, self.py)
        self.depth_image = np.asanyarray(self.depth_frame.get_data())
        self.color_image = np.asanyarray(self.color_frame.get_data())
        
    def processFoto(self, angle):
        print(angle)
        self.angle = angle
        self.depth_frame_open3d = o3d.geometry.Image(self.depth_image)
        self.color_frame_open3d = o3d.geometry.Image(self.color_image)

        self.rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(self.color_frame_open3d, self.depth_frame_open3d, convert_rgb_to_intensity=False)
        self.pcd = o3d.geometry.PointCloud.create_from_rgbd_image(self.rgbd_image, self.intrinsic)
        self.pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        self.pcd.orient_normals_towards_camera_location(camera_location=np.array([0., 0., 0.]))
        self.getcameraLocation()
        self.rMatrix()
        self.pcd.rotate(self.R, (0, 0, 0))
        self.pcd.translate((self.x, self.y, self.z))
        self.pcd = self.pcd.crop(self.bbox)
        self.pcd, self.ind = self.pcd.remove_statistical_outlier(nb_neighbors=100, std_ratio=2)
        
        if len(self.main_pcd.points) > 0:
            transformation, fitness, inlier_rmse = self.applyICP(self.pcd, self.main_pcd)
            self.pcd.transform(transformation)
            
            # Save transformed point cloud and metrics
            if not hasattr(self, 'all_transformations'):
                self.all_transformations = []
            self.all_transformations.append((self.pcd, transformation, fitness, inlier_rmse))
        
        else:
            self.main_pcd = self.pcd
        
    def getPointcloud(self):
        return self.main_pcd
    
    def giveImageArray(self):
        return self.color_image
    
    def getcameraLocation(self):
        self.x = np.sin(self.angle * self.dtr) * self.distance - np.cos(self.angle * self.dtr) * -0.1
        self.y = -np.cos(self.angle * self.dtr) * self.distance - np.sin(self.angle * self.dtr) * 0.0001
        self.z = 0.200
        self.o = self.angle
        self.a = 112.5
        self.t = 0
    
    def rMatrix(self):
        self.o = self.o * self.dtr
        self.a = (-self.a) * self.dtr
        self.t = self.t * self.dtr
        self.R = [[np.cos(self.o) * np.cos(self.t) - np.cos(self.a) * np.sin(self.o) * np.sin(self.t), -np.cos(self.o) * np.sin(self.t) - np.cos(self.a) * np.cos(self.t) * np.sin(self.o), np.sin(self.o) * np.sin(self.a)],
                  [np.cos(self.t) * np.sin(self.o) + np.cos(self.o) * np.cos(self.a) * np.sin(self.t), np.cos(self.o) * np.cos(self.a) * np.cos(self.t) - np.sin(self.o) * np.sin(self.t), -np.cos(self.o) * np.sin(self.a)],
                  [np.sin(self.a) * np.sin(self.t), np.cos(self.t) * np.sin(self.a), np.cos(self.a)]]
    
    def applyICP(self, source_pcd, target_pcd):
        # Try different threshold values
        thresholds = [0.02, 0.02, 0.03, 0.04]
        
        best_transformation = None
        best_fitness = -1  # Initial value to compare fitness
        best_rmse = float('inf')  # Initial value to compare RMSE
        
        for threshold in thresholds:
            trans_init = np.eye(4)  # Initial transformation
            print(f"Evaluating ICP with threshold: {threshold}")
            
            evaluation = o3d.pipelines.registration.evaluate_registration(source_pcd, target_pcd, threshold, trans_init)
            print("Initial evaluation:", evaluation)
            
            reg_p2p = o3d.pipelines.registration.registration_icp(
                source_pcd, target_pcd, threshold, trans_init,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
            
            print(f"ICP converged with threshold {threshold}. Fitness: ", reg_p2p.fitness, " Inlier RMSE: ", reg_p2p.inlier_rmse)
            print("Transformation matrix:\n", reg_p2p.transformation)
            
            if reg_p2p.fitness > best_fitness or (reg_p2p.fitness == best_fitness and reg_p2p.inlier_rmse < best_rmse):
                best_transformation = reg_p2p.transformation
                best_fitness = reg_p2p.fitness
                best_rmse = reg_p2p.inlier_rmse
        
        print("Best ICP result: Fitness:", best_fitness, "Inlier RMSE:", best_rmse)
        return best_transformation, best_fitness, best_rmse
    
    def finalizePointCloud(self):
        if hasattr(self, 'all_transformations'):
            best_pcd, best_transformation, best_fitness, best_rmse = max(self.all_transformations, key=lambda x: x[2])
            self.main_pcd = best_pcd
            print("Selected best point cloud with fitness:", best_fitness, "and inlier RMSE:", best_rmse)
        return self.main_pcd
    
    def makeSTL(self, kpoints, stdRatio, depth, iterations):
        self.finalizePointCloud()  # Finalize point cloud before creating mesh
        self.stl_pcd = self.main_pcd
        self.stl_pcd = self.stl_pcd.uniform_down_sample(every_k_points=kpoints)
        self.stl_pcd, self.ind = self.stl_pcd.remove_statistical_outlier(nb_neighbors=100, std_ratio=stdRatio)
        self.bbox1 = o3d.geometry.AxisAlignedBoundingBox((-0.13, -0.13, 0), (0.13, 0.13, 0.01))
        self.bottom = self.stl_pcd.crop(self.bbox1)
        
        try:
            self.hull, _ = self.bottom.compute_convex_hull()  
            self.boundary_vertices = self.hull.compute_boundary_vertices()
            
            # Generate random data to add to boundary vertices
            random_data = np.random.rand(len(self.boundary_vertices), 3) * 0.1  # Random data between 0 and 0.1
            
            # Add random data to boundary vertices
            for i, vertex_idx in enumerate(self.boundary_vertices):
                self.bottom.vertices[vertex_idx] += random_data[i]
            
            self.bottom.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
            self.bottom.orient_normals_towards_camera_location(camera_location=np.array([0., 0., -10.]))
            self.bottom.paint_uniform_color([0, 0, 0])
            _, self.pt_map = self.bottom.hidden_point_removal([0, 0, -1], 1)
            self.bottom = self.bottom.select_by_index(self.pt_map)
            self.stl_pcd = self.stl_pcd + self.bottom
        
        except Exception as e:
            print("Error creating bottom part of mesh:", e)
        
        finally:
            self.mesh, self.densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(self.stl_pcd, depth=depth)
            self.mesh = self.mesh.filter_smooth_simple(number_of_iterations=iterations)
            self.mesh.scale(1000, center=(0, 0, 0))
            self.mesh.compute_vertex_normals()
        
        return self.mesh




class App(tk.Tk):

    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)
        self.processed = False  # Variable para indicar si la imagen ya ha sido procesada
        self.photo_counter = 0  # Contador de fotos


        self.title_font = tkfont.Font(family='Arial', size=18, weight="bold", slant="italic")
        self.title("R3Dsystem")
        self.angle = 0
        # self.wm_iconbitmap('icoontje3dscan_WDJ_icon.ico')
        self.iconbitmap(default='icoontje3dscan_WDJ_icon.ico')
        #self.resizable(False,False)
        self.dictionary = self.readSettings()
        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}
        for F in (StartPage, SettingsPage):
            page_name = F.__name__
            frame = F(parent=container, controller=self)
            self.frames[page_name] = frame
            frame.grid(row=0, column=0, sticky="nsew")

        self.show_frame("StartPage")
        
        #self.frames["StartPage"].stlButton.configure(bg = "#cccccc")
        #self.frames["StartPage"].saveButton.configure(bg = "#cccccc")
        self.frames["StartPage"].buttonShowPC.configure(bg = "#cccccc")
        self.enablePC = False
        self.enableSaveSTL = False

        #Prueba de captura de imagenes
   


    def tomar_foto(self, q1, numerototal):
        if self.photo_counter == 0:  # Inicia la barra de progreso la primera vez
            self.frames["StartPage"].startProgress()
        
        print("Tomando foto...")
        print(q1)

        print("Foto tomada, enviando señal para continuar...")
        # Descomentar cuando la camara esté en funcionamiento
        self.scan.takeFoto()
        self.frames["StartPage"].showImage(self.scan.giveImageArray())
        angle = q1
        print("Se ejecutó el número total de fotos")
        print(angle)
        
        self.photo_counter += 1  # Incrementa el contador de fotos
        print(f"Fotos tomadas: {self.photo_counter}")  # Imprime el contador en la consola

        # Actualiza la barra de progreso basado en el número total de fotos
        progress_value = (self.photo_counter / numerototal) * 360
        self.frames["StartPage"].Progress( progress_value)

        self.scan.processFoto(angle)
        self.update()
    
        


   
       
    
        
    def show_frame(self, page_name):
        '''Show a frame for the given page name'''
        frame = self.frames[page_name]
        frame.tkraise()
        
    def startScan(self):
        Asimuth, Altitud, Roll, sphere_radius, selected_option, radio_var = self.frames["StartPage"].getSliderData()

        if Asimuth == 0 or  Altitud == 0 or Roll == 0 or sphere_radius == 0:
            raise ValueError("Alguno de los valores es igual a cero") 
        else:
            print(radio_var)
            if (radio_var=="Option 1"): 
                self.scan = Scan(int(self.dictionary["widthFrame"]),int(self.dictionary["heightFrame"]),30,10,0)
                self.scan.startPipeline()
                prueba_1.procesarDatos(Altitud, Asimuth,Roll,sphere_radius, lambda q1,numerototal: self.tomar_foto(q1,numerototal))
                print('se termino de ejectar la funcion de los datos  ')
                self.scan.stopPipeline()
                self.enablePC = True

            else:
                if (selected_option=="Option 36"):
                    resultado = prueba_1.Select_Imagenes_modo_2(36)
                    prueba_1.procesarDatos(resultado[0],resultado[1],resultado[2])
                    ##datos_recibidos = prueba_1.recibir_datos_por_puerto_serie()
                if (selected_option=="Option 64"):
                    resultado = prueba_1.Select_Imagenes_modo_2(64)
                    prueba_1.procesarDatos(resultado[0],resultado[1],resultado[2])
                    #datos_recibidos = prueba_1.recibir_datos_por_puerto_serie()



        self.ard = Arduino(str(self.dictionary["COMport"]),int(self.dictionary["baudrate"]),0.1)
        self.scan = Scan(int(self.dictionary["widthFrame"]),int(self.dictionary["heightFrame"]),30,10,0)
        self.scan.startPipeline()
        self.frames["StartPage"].startProgress()
        
        try:
            while True:
                
                self.scan.takeFoto()
                self.frames["StartPage"].showImage(self.scan.giveImageArray())              
                angle = float(self.ard.giveAngle())
                self.frames["StartPage"].Progress(angle)               
                self.ard.rotate(int(self.dictionary["stepSize"]))
                self.scan.processFoto(angle)
                self.ard.waitForRotation()    
                self.update()
                if angle >= 360:
                    print("de cirkel is rond!")
                    self.frames["StartPage"].endProgress()
                    break
                
                if keyboard.is_pressed('q'):
                    print("hij stopt")
                    break
        except:
            print("loop is kapot") 
            pass
        finally:      
            self.scan.stopPipeline()
            self.ard.close()
            self.enablePC = True
            self.frames["StartPage"].stlButton.configure(bg = "#f2f2f2")      
            self.frames["StartPage"].buttonShowPC.configure(bg = "#f2f2f2")
   
    def showPC(self):     
        o3d.visualization.draw_geometries([self.scan.getPointcloud()])
        
    def makeSTL(self):
        if self.enablePC == True:
            self.STL = self.scan.makeSTL(int(self.dictionary["k_points"]),float(self.dictionary["std_ratio"]),int(self.dictionary["depth"]),int(self.dictionary["iterations"]))
            o3d.visualization.draw_geometries([self.STL])
            self.enableSaveSTL = True
            self.frames["StartPage"].saveButton.configure(bg = "#f2f2f2")
            print("makestl")
        
    def saveSTL(self):
        if self.enableSaveSTL == True:
            self.directory = tk.filedialog.asksaveasfilename(initialfile="MySTL.stl")      
            o3d.io.write_triangle_mesh(self.directory, self.STL)
     
    def readSettings(self):
        file = open("settings.txt", "r")
        contents = file.read()
        dictionary = ast.literal_eval(contents)
        file.close()
        return dictionary
        
    def writeSettings(self,newdictionary):
        self.newdictionary = newdictionary
        with open("settings.txt",'w') as self.data:
            self.data.write(str(self.newdictionary))
        
    def saveSettings(self):
        self.frames["SettingsPage"].enterSettings()
        self.writeSettings(self.dictionary)
        
    def close_windows(self):     
        self.destroy()
        
        
class StartPage(tk.Frame):
    
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        
        self.controller = controller
            
        self.buttonFrame = tk.Frame(self) #,highlightbackground="black",highlightthickness=1
        self.buttonStart = tk.Button(self.buttonFrame, text = 'Iniciar Captura', width = 25, command = self.controller.startScan)
        self.buttonStart.grid(sticky="W",row = 0, column = 5, pady = 5, padx = 10)
        
        self.buttonSettings = tk.Button(self.buttonFrame, text = 'Configuracion', width = 25, command=lambda: controller.show_frame("SettingsPage"))
        self.buttonSettings.grid(sticky="W",row = 1, column = 5, pady = 5, padx = 10)
        
        self.buttonShowPC = tk.Button(self.buttonFrame, text = 'Mostrar modelo', width = 25, command = self.controller.showPC)
        self.buttonShowPC.grid(sticky="W",row = 2, column = 5, pady = 5, padx = 10)
             
        #self.stlButton = tk.Button(self.buttonFrame, text = 'Convertir a STL', width = 25, command = self.controller.makeSTL)
        #self.stlButton.grid(sticky="W",row = 3, column =5, pady = 5, padx = 10)
        
        #self.saveButton = tk.Button(self.buttonFrame, text = 'Guardar STL', width = 25, command = self.controller.saveSTL)
        #self.saveButton.grid(sticky="W",row = 4, column = 5, pady = 5, padx = 10)
        
        self.quitButton = tk.Button(self.buttonFrame, text = 'Salir', width = 25, command = self.controller.close_windows)
        self.quitButton.grid(sticky="W",row = 5, column = 5, pady = 5, padx = 10)


        #Sliders
        
        label1 = tk.Label(self.buttonFrame, text="Posicion azimut")
        label1.grid(sticky="W", row=0, column=0, pady=5, padx=10)
        self.sliderAzimuth = tk.Scale(self.buttonFrame, from_=0, to=100, orient='horizontal', length=200)
        self.sliderAzimuth.grid(sticky="W", row=0, column=1, pady=5, padx=10)


        label2 = tk.Label(self.buttonFrame, text="Posicion Altitud")
        label2.grid(sticky="W", row=1, column=0, pady=5, padx=10)
        self.sliderAltitude = tk.Scale(self.buttonFrame, from_=0, to=100, orient='horizontal', length=200)
        self.sliderAltitude.grid(sticky="W", row=1, column=1, pady=5, padx=10)

        label3 = tk.Label(self.buttonFrame, text="Radio de la esfera")
        label3.grid(sticky="W", row=2, column=0, pady=5, padx=10)
        self.sliderRoll = tk.Scale(self.buttonFrame, from_=0, to=270, orient='horizontal', length=200)
        self.sliderRoll.grid(sticky="W", row=2, column=1, pady=5, padx=10)

        
        label4 = tk.Label(self.buttonFrame, text="Radio de la Roll")
        label4.grid(sticky="W", row=3, column=0, pady=5, padx=10)
        self.sliderSphereRadius = tk.Scale(self.buttonFrame, from_=0, to=100, orient='horizontal', length=200)
        self.sliderSphereRadius.grid(sticky="W", row=3, column=1, pady=5, padx=10)

       


    
        self.selectLabel = tk.Label(self.buttonFrame, text="Seleccionar una opcion:")
        self.selectLabel.grid(sticky="W", row=0, column=4, pady=5, padx=10)

        self.options = ["Option 36","Option 64"]
        self.selectedOption = tk.StringVar()
        self.selectMenu = tk.OptionMenu(self.buttonFrame, self.selectedOption, *self.options)
        self.selectMenu.grid(sticky="W", row=1, column=4, pady=5, padx=10)


        self.radioLabel = tk.Label(self.buttonFrame, text="Selecciona modo de captura:")
        self.radioLabel.grid(sticky="W", row=0, column=2, pady=5, padx=10)

        self.radioVar = tk.StringVar()
        self.radioVar.set("Modo 1")
        self.radio1 = tk.Radiobutton(self.buttonFrame, text="Modo 1", variable=self.radioVar, value="Option 1")
        self.radio1.grid(sticky="W", row=1, column=2, pady=5, padx=10)

        self.radio2 = tk.Radiobutton(self.buttonFrame, text="Modo 2", variable=self.radioVar, value="Option 2")
        self.radio2.grid(sticky="W", row=2, column=2, pady=5, padx=10)

        



        self.buttonFrame.grid(sticky="W",row = 0, column = 1)
        
        self.load = Image.fromarray(np.zeros(shape=(480,848,3)), 'RGB')
        self.left,self.upper,self.right,self.lower = 330,20,670,480
 
        # self.load = Image.open("testfoto.png")
        self.render = ImageTk.PhotoImage(self.load.crop((self.left,self.upper,self.right,self.lower)))
        self.canvas = tk.Canvas(self, width = self.right-self.left, height = self.lower-self.upper)  
        self.canvas.grid(sticky="s",row = 0, column = 2)
        self.canvas.create_image(0,0, anchor='nw', image=self.render)    
        self.canvas.image = self.render  



    def getSliderData(self):
            azimuth = self.sliderAzimuth.get()
            altitude = self.sliderAltitude.get()
            roll = self.sliderRoll.get()
            sphere_radius = self.sliderSphereRadius.get()
            selected_option = self.selectedOption.get()
            radio_var = self.radioVar.get()
            
            return azimuth, altitude, roll, sphere_radius, selected_option, radio_var


    def showImage(self,iArray):
        self.load = Image.fromarray(iArray, 'RGB')
        self.render = ImageTk.PhotoImage(self.load.crop((self.left,self.upper,self.right,self.lower)))
        self.canvas.create_image(0,0, anchor='nw', image=self.render)    
        self.canvas.image = self.render
    
    def startProgress(self):
        self.progress = Progressbar(self,orient="horizontal",length=self.right-self.left,mode='determinate',maximum = 360)
        self.progress.grid(sticky="W",row = 1, column = 1, pady = 5) 
    
    def Progress(self,getal):
        self.progress['value'] = getal
    
    def endProgress(self):
        self.progress.grid_forget()  
        
class SettingsEntry():
    
    def __init__(self,master,name, **kwargs):
        self.name = name
        self.master = master
        self.var = kwargs.get('var', None)
        self.frame = tk.Frame(self.master)
        self.label = tk.Label(self.frame,text = self.name,width = 15, anchor='e')
        self.label.pack(side = "left",fill = "both")
        self.entry = tk.Entry(self.frame)
        self.entry.pack(side = "left",fill = "both")
        self.entry.delete(0)
        self.entry.insert(0,self.var)
        self.frame.pack()
        
    def get(self):       
        return self.entry.get()
    
    def insert(self,newVar):
        self.entry.delete(0,'end')
        self.entry.insert(0,newVar)
        
class SettingsPage(tk.Frame):
    
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        
        self.controller = controller
        
        self.buttonFrame = tk.Frame(self)
        self.buttonMp = tk.Button(self.buttonFrame, text = 'Main Page', width = 25, command=lambda: controller.show_frame("StartPage"))
        self.buttonMp.grid(sticky="W",row = 0, column = 0, pady = 5, padx = 10)
        
        # self.buttonEntersettings = tk.Button(self.buttonFrame, text="Use Settings",width = 25,command=self.enterSettings)
        # self.buttonEntersettings.grid(sticky="W",row = 1, column = 0, pady = 5, padx = 10)
        
        self.buttonDefaultsettings = tk.Button(self.buttonFrame, text="Reset Default Settings",width = 25,command=self.defaultSettings)
        self.buttonDefaultsettings.grid(sticky="W",row = 1, column = 0, pady = 5, padx = 10)
        
        self.buttonSavesettings = tk.Button(self.buttonFrame, text="Save and Use Settings",width = 25,command=self.controller.saveSettings)
        self.buttonSavesettings.grid(sticky="W",row = 2, column = 0, pady = 5, padx = 10)
        
        self.quitButton = tk.Button(self.buttonFrame, text = 'Quit', width = 25, command = self.controller.close_windows)
        self.quitButton.grid(sticky="W",row = 3, column = 0, pady = 5, padx = 10)
        
        self.buttonFrame.pack(side = "left")
        
        self.BasicSettingsFrame = tk.LabelFrame(self,text = "Basic Settings")
        
        self.e1 = SettingsEntry(self.BasicSettingsFrame,"Step size: ",var = self.controller.dictionary["stepSize"])
        self.e2 = SettingsEntry(self.BasicSettingsFrame,"Frame width: ",var = self.controller.dictionary["widthFrame"])
        self.e3 = SettingsEntry(self.BasicSettingsFrame,"Frame height: ",var = self.controller.dictionary["heightFrame"])
        #self.e4 = SettingsEntry(self.BasicSettingsFrame,"COM port Arduino: ",var = self.controller.dictionary["COMport"])
        self.e5 = SettingsEntry(self.BasicSettingsFrame,"Baudrate Arduino: ",var = self.controller.dictionary["baudrate"])
        
        self.BasicSettingsFrame.pack(side = "left",fill="both")#fill="both", expand="yes"
        
        self.MeshSettingsFrame = tk.LabelFrame(self,text = "Mesh Settings")
        
        self.e6 = SettingsEntry(self.MeshSettingsFrame,"k points: ",var = self.controller.dictionary["k_points"])
        self.e7 = SettingsEntry(self.MeshSettingsFrame,"std ratio: ",var = self.controller.dictionary["std_ratio"])
        self.e8 = SettingsEntry(self.MeshSettingsFrame,"depth: ",var = self.controller.dictionary["depth"])
        self.e9 = SettingsEntry(self.MeshSettingsFrame,"iterations: ",var = self.controller.dictionary["iterations"])
        
        self.MeshSettingsFrame.pack(side = "left",fill="both")

    def enterSettings(self):
        self.controller.dictionary["stepSize"] = self.e1.get()
        self.controller.dictionary["widthFrame"] = self.e2.get()
        #self.controller.dictionary["heightFrame"] = self.e3.get()
        self.controller.dictionary["COMport"] = self.e4.get()
        self.controller.dictionary["baudrate"] = self.e5.get()
        self.controller.dictionary["k_points"] = self.e6.get()
        self.controller.dictionary["std_ratio"] = self.e7.get()
        self.controller.dictionary["depth"] = self.e8.get()
        self.controller.dictionary["iterations"] = self.e9.get()
        
    def defaultSettings(self):
        # het kan ook direct in de insert, maar zo (met de tussenstap) past ie ook gelijk de dictionary aan
        self.controller.dictionary["stepSize"] = 256
        self.controller.dictionary["widthFrame"] = 848
        self.controller.dictionary["heightFrame"] = 480
        self.controller.dictionary["COMport"] = "COM3"
        self.controller.dictionary["baudrate"] = 9600
        self.controller.dictionary["k_points"] = 10
        self.controller.dictionary["std_ratio"] = 0.5
        self.controller.dictionary["depth"] = 7
        self.controller.dictionary["iterations"] = 8
        self.e1.insert(self.controller.dictionary["stepSize"])
        self.e2.insert(self.controller.dictionary["widthFrame"])
        self.e3.insert(self.controller.dictionary["heightFrame"])
        #self.e4.insert(self.controller.dictionary["COMport"])
        self.e5.insert(self.controller.dictionary["baudrate"])
        self.e6.insert(self.controller.dictionary["k_points"])
        self.e7.insert(self.controller.dictionary["std_ratio"])
        self.e8.insert(self.controller.dictionary["depth"])
        self.e9.insert(self.controller.dictionary["iterations"])
    
if __name__ == "__main__":
    app = App()
    app.mainloop()