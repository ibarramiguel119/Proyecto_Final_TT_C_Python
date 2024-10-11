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
import threading
from tkinter import Toplevel
from tkinter import ttk, filedialog, messagebox
import sqlite3
import os
import cv2
import time
from PIL import Image, ImageTk  # Importa Pillow para cargar imágenes


class Gestor:
    def __init__(self):
        # Inicializar la configuración del pipeline para capturar imágenes
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Inicializar variables para los frames y canvas
        self.canvas_imagenes = None
        self.canvas_frame = None
        self.frame_directorios = None

    def capturar_imagenes_secuenciales(self):
        # Iniciar la captura de imágenes
        self.pipeline.start(self.config)
        try:
            # Crear la carpeta donde se guardarán las imágenes
            carpeta_principal = "Objetos_Capturados"
            os.makedirs(carpeta_principal, exist_ok=True)

            # Crear una subcarpeta con la fecha y hora actual
            nombre_carpeta = time.strftime("Objeto_%Y%m%d_%H%M%S")
            ruta_carpeta = os.path.join(carpeta_principal, nombre_carpeta)
            os.makedirs(ruta_carpeta, exist_ok=True)

            # Capturar 10 imágenes
            for i in range(10):
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()

                if not color_frame:
                    continue

                # Convertir los datos del frame a un array de imagen
                color_image = np.asanyarray(color_frame.get_data())
                ruta = os.path.join(ruta_carpeta, f"Imagen_{i+1}.jpg")
                cv2.imwrite(ruta, color_image)

                print(f"Imagen capturada: {i + 1}")
                time.sleep(0.5)  # Espera entre capturas

            messagebox.showinfo("Éxito", f"10 imágenes capturadas y guardadas en la carpeta {ruta_carpeta}.")
            self.crear_arbol_directorios()

        except Exception as e:
            messagebox.showerror("Error", f"No se pudo capturar la imagen: {e}")

        finally:
            self.pipeline.stop()

    def crear_arbol_directorios(self):
        # Verificar si la carpeta existe
        path = os.path.join(os.getcwd(), "Objetos_Capturados")

        if not os.path.exists(path):
            messagebox.showerror("Error", f"La carpeta {path} no existe.")
            return

        # Limpiar el contenido actual del frame
        for widget in self.frame_directorios.winfo_children():
            widget.destroy()

        # Crear el Treeview para mostrar los directorios
        tree = ttk.Treeview(self.frame_directorios, show="tree")
        tree.pack(fill=tk.BOTH, expand=True)

        def agregar_elementos_arbol(parent, ruta):
            try:
                items = os.listdir(ruta)
            except PermissionError:
                return  # Omitir carpetas sin permiso

            for item in items:
                item_path = os.path.join(ruta, item)
                if os.path.isdir(item_path):
                    oid = tree.insert(parent, 'end', text=item, open=False, values=[item_path])
                    agregar_elementos_arbol(oid, item_path)
                else:
                    tree.insert(parent, 'end', text=item, open=False, values=[item_path])

        # Insertar el directorio raíz
        root_id = tree.insert('', 'end', text=os.path.basename(path), open=True, values=[path])
        agregar_elementos_arbol(root_id, path)

        # Función que se ejecuta al seleccionar un directorio en el Treeview
        

        def on_tree_select(event):
            selected_item = tree.selection()
            print(f"Selected Item: {selected_item}")  # Debugging print
            
            if selected_item:
                item_data = tree.item(selected_item[0])
                print(f"Item Data: {item_data}")  # Debugging print
                
                if 'values' in item_data:
                    print(f"Item Values: {item_data['values']}")  # Debugging print
                    
                    if item_data['values']:
                        ruta_carpeta = item_data['values'][0]
                        if os.path.isdir(ruta_carpeta):
                            self.mostrar_imagenes_carpeta(ruta_carpeta)
                        else:
                            # If a file is selected, get its directory
                            carpeta = os.path.dirname(ruta_carpeta)
                            self.mostrar_imagenes_carpeta(carpeta)
                    else:
                        print("No values associated with the selected item.")
                else:
                    print("No 'values' key found in item_data.")
            else:
                print("No item selected.")
        tree.bind('<<TreeviewSelect>>', on_tree_select)      






    def mostrar_imagenes_carpeta(self, ruta_carpeta):
        # Limpiar el canvas antes de mostrar nuevas imágenes
        for widget in self.canvas_imagenes.winfo_children():
            widget.destroy()

        # Verificar si la carpeta existe
        if not os.path.exists(ruta_carpeta):
            messagebox.showerror("Error", f"La carpeta {ruta_carpeta} no existe.")
            return

        # Listar imágenes en la carpeta
        imagenes = [f for f in os.listdir(ruta_carpeta) if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif'))]

        if imagenes:
            for idx, nombre in enumerate(imagenes):
                ruta = os.path.join(ruta_carpeta, nombre)
                if os.path.exists(ruta):
                    try:
                        # Cargar y redimensionar la imagen
                        img = Image.open(ruta)
                        img.thumbnail((100, 100))
                        img_tk = ImageTk.PhotoImage(img)

                        # Mostrar la imagen en el canvas
                        label = tk.Label(self.canvas_imagenes, image=img_tk, text=nombre, compound=tk.TOP)
                        label.image = img_tk
                        label.grid(row=idx//4, column=idx%4, padx=5, pady=5)
                    except Exception as e:
                        print(f"No se pudo cargar la imagen {nombre}: {e}")
                else:
                    print(f"No se encontró la imagen en la ruta: {ruta}")
        else:
            messagebox.showinfo("Sin imágenes", "No se encontraron imágenes en esta carpeta.")

        # Ajustar el scroll del canvas
        self.canvas_imagenes.update_idletasks()
        self.canvas_frame.config(scrollregion=self.canvas_imagenes.bbox("all"))

    def open_new_window(self):
        # Crear una nueva ventana para mostrar el gestor
        new_window = tk.Toplevel()
        new_window.title("Gestor de Objetos")
        new_window.geometry("1100x600")

        # Frame para los directorios
        self.frame_directorios = tk.Frame(new_window)
        self.frame_directorios.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # Frame para las imágenes
        frame_imagenes = tk.Frame(new_window)
        frame_imagenes.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        # Canvas para las imágenes
        self.canvas_frame = tk.Canvas(frame_imagenes)
        self.canvas_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Scrollbar para el canvas
        scrollbar = tk.Scrollbar(frame_imagenes, orient=tk.VERTICAL, command=self.canvas_frame.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # Asociar el scrollbar al canvas
        self.canvas_frame.config(yscrollcommand=scrollbar.set)

        # Frame interno para contener las imágenes dentro del canvas
        self.canvas_imagenes = tk.Frame(self.canvas_frame)
        self.canvas_frame.create_window((0, 0), window=self.canvas_imagenes, anchor="nw")

        # Asociar la función de ajuste de tamaño del canvas
        self.canvas_imagenes.bind("<Configure>", lambda event: self.canvas_frame.config(scrollregion=self.canvas_imagenes.bbox("all")))

        # Botón para iniciar la captura de imágenes
        btn_iniciar_captura = tk.Button(new_window, text="Iniciar Captura", command=self.capturar_imagenes_secuenciales)
        btn_iniciar_captura.grid(row=1, column=0, padx=10, pady=10)

        # Inicializar el árbol de directorios
        self.crear_arbol_directorios()

      
  


class Scan():
    def __init__(self, width, height, framerate, autoexposureFrames, backDistance,ruta_carpeta):
        self.width = width
        self.height = height
        self.framerate = framerate
        self.backDistance = backDistance
        self.autoexposureFrames = autoexposureFrames
        self.main_pcd = o3d.geometry.PointCloud()

        self.contador_imagenes = 0
        self.ruta_carpeta = ruta_carpeta
        
        
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
        
        # Ajustar autoexposición
        for i in range(self.autoexposureFrames):
            self.frameset = self.pipe.wait_for_frames()
        
        # Capturar el frameset actual
        self.frameset = self.pipe.wait_for_frames()
        self.frameset = self.align.process(self.frameset)
        
        # Obtener intrínsecos de la cámara de profundidad
        self.profile = self.frameset.get_profile()
        self.depth_intrinsics = self.profile.as_video_stream_profile().get_intrinsics()
        self.w, self.h = self.depth_intrinsics.width, self.depth_intrinsics.height
        self.fx, self.fy = self.depth_intrinsics.fx, self.depth_intrinsics.fy
        self.px, self.py = self.depth_intrinsics.ppx, self.depth_intrinsics.ppy
        
        # Obtener los frames de color y profundidad
        self.color_frame = self.frameset.get_color_frame()
        self.depth_frame = self.frameset.get_depth_frame()
        
        # Verificar si los frames son válidos
        if not self.color_frame or not self.depth_frame:
            print("Error: no se pudieron obtener los frames de color o profundidad.")
            return
        
        # Convertir los datos del frame a arrays de imagen
        self.depth_image = np.asanyarray(self.depth_frame.get_data())
        self.color_image = np.asanyarray(self.color_frame.get_data())
        
        # Guardar la imagen capturada en la carpeta creada por `startScaner`
        ruta_imagen = os.path.join(self.ruta_carpeta, f"Imagen_{self.contador_imagenes}.jpg")

        

        cv2.imwrite(ruta_imagen, self.color_image)
        
        print(f"Imagen capturada y guardada en {ruta_imagen}")
        
        # Incrementar el contador para la siguiente imagen
        self.contador_imagenes += 1
        
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
            #print(f"Evaluating ICP with threshold: {threshold}")

            
            
            evaluation = o3d.pipelines.registration.evaluate_registration(source_pcd, target_pcd, threshold, trans_init)
            #print("Initial evaluation:", evaluation)
            
            reg_p2p = o3d.pipelines.registration.registration_icp(
                source_pcd, target_pcd, threshold, trans_init,
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
            
            #print(f"ICP converged with threshold {threshold}. Fitness: ", reg_p2p.fitness, " Inlier RMSE: ", reg_p2p.inlier_rmse)
            #print("Transformation matrix:\n", reg_p2p.transformation)
            
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
            #print("Selected best point cloud with fitness:", best_fitness, "and inlier RMSE:", best_rmse)
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

        self.dictionary = self.readSettings()
        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}
        self.gestor_instance = Gestor()  # Create an instance of Gestor

        for F in (StartPage, SettingsPage):
            page_name = F.__name__
            frame = F(parent=container, controller=self, gestor_instance=self.gestor_instance)
            self.frames[page_name] = frame
            frame.grid(row=0, column=0, sticky="nsew")

        self.show_frame("StartPage")
        
        self.frames["StartPage"].buttonShowPC.configure(bg="#cccccc")
        self.enablePC = False
        self.enableSaveSTL = False

    def tomar_foto(self, q1, numerototal):
        self.update()
        if self.photo_counter == 0:  # Inicia la barra de progreso la primera vez
            self.frames["StartPage"].startProgress()
            self.photo_counter=0

        #self.frames["StartPage"].log(f"Numero total de imagenes: {numerototal}") 
        self.update()   
        self.frames["StartPage"].log("Tomando foto...")
        self.update()
        self.frames["StartPage"].log("Foto tomada, enviando señal para continuar...")
        self.update()
        
        self.scan.takeFoto()
        self.update()
        self.frames["StartPage"].showImage(self.scan.giveImageArray())
        angle = q1
        #self.frames["StartPage"].log("Se ejecutó el número total de fotos")
        #self.update()
        
        self.update()
        
        
        # Actualiza la barra de progreso basado en el número total de fotos
        progress_value = (self.photo_counter / numerototal) * 360
        self.frames["StartPage"].Progress(progress_value)
        self.update()
        #self.scan.processFoto(angle)
        self.frames["StartPage"].log(f"Cordenada articular q1: {angle}")  # Imprime el contador en la consola
        self.update()
        
        if progress_value >360:
            self.photo_counter = 0  # Reinicia el contador de fotos
            progress_value = 0  # Reinicia la barra de progreso
            self.frames["StartPage"].log("Proceso terminado existosamente.")
            self.update()
        self.photo_counter += 1  # Incrementa el contador de fotos
        
        

    def show_frame(self, page_name):
        '''Show a frame for the given page name'''
        frame = self.frames[page_name]
        frame.tkraise()
        
    def startScan(self):
        Asimuth, Altitud, Roll, New_roll, selected_option, radio_var = self.frames["StartPage"].getSliderData()
        self.frames["StartPage"].log(f"Modo seleccionado: {radio_var}")

        # Crear la carpeta principal y una subcarpeta con la fecha y hora actual
        self.carpeta_principal = "Objetos_Capturados"
        os.makedirs(self.carpeta_principal, exist_ok=True)
        
        # Crear la subcarpeta con la fecha y hora actual, pero solo una vez al inicio
        nombre_carpeta = time.strftime("Objeto_%Y%m%d_%H%M%S")
        self.ruta_carpeta = os.path.join(self.carpeta_principal, nombre_carpeta)
        os.makedirs(self.ruta_carpeta, exist_ok=True)


        print(f"Ruta de la carpeta principal: {self.carpeta_principal}")
        print(f"Ruta completa de la carpeta: {self.ruta_carpeta}")
        
        # Inicializar contador de imágenes
        self.contador_imagenes = 0
        self.frames["StartPage"].log(f"Escaneo inicido en direcion: {self.ruta_carpeta}")
        

        if radio_var == "Option 1":
            if Asimuth == 0 or Altitud == 0 or Roll == 0:
                self.frames["StartPage"].log('Algunos de los valores es igul a cero')
                self.frames["StartPage"].log('No se permiten selecionar 0 imagenes')
                raise ValueError("Alguno de los valores es igual a cero") 
            
            else:
                self.frames["StartPage"].log('Inicializando sistema...')
                self.update()
                try:
                    print(type(self.ruta_carpeta))
                    self.scan = Scan(int(self.dictionary["widthFrame"]), int(self.dictionary["heightFrame"]), 30, 10, 0,self.ruta_carpeta)
                    self.scan.startPipeline()
                    prueba_1.procesarDatos(Altitud, Asimuth, Roll, New_roll, lambda q1, numerototal: self.tomar_foto(q1, numerototal))
                    
                    self.frames["StartPage"].log('Se terminó de ejecutar la función de los datos')
                    self.update()
                    self.frames["StartPage"].log('Esperando que se termine de evaluar la nube de puntos')
                    self.update()
                    self.scan.stopPipeline()
                    self.frames["StartPage"].log('Sistema listo para mostrar el modelo')
                    self.frames["StartPage"].Progress(360)
                    self.update()
                    self.enablePC = True    

                except Exception as e:
                    self.frames["StartPage"].log(f"Error al inicializar la cámara: {str(e)}")
                    self.scan = None  # Asegúrate de que la variable scan se restablezca si hay un error
                self.update()
        else:
            if selected_option == "27 imágenes":
                self.frames["StartPage"].log('Inicializando sistema...')
                self.update()
                try:
                    self.scan = Scan(int(self.dictionary["widthFrame"]), int(self.dictionary["heightFrame"]), 30, 10, 0)
                    self.scan.startPipeline()
                    resultado = prueba_1.Select_Imagenes_modo_2(27)
                    prueba_1.procesarDatos(resultado[0], resultado[1], resultado[2],3, lambda q1, numerototal: self.tomar_foto(q1, numerototal))
                    self.frames["StartPage"].log('Se terminó de ejecutar la función de los datos')
                    self.update()
                    self.frames["StartPage"].log('Esperando que se termine de evaluar la nube de puntos')
                    self.update()
                    self.scan.stopPipeline()
                    self.frames["StartPage"].log('Sistema listo para mostrar el modelo')
                    self.frames["StartPage"].Progress(360)
                    self.update()
                    self.enablePC = True
                except Exception as e:
                    self.frames["StartPage"].log(f"Error al inicializar la cámara: {str(e)}")
                    self.scan = None  # Asegúrate de que la variable scan se restablezca si hay un error
                self.update()

            if selected_option == "50 imágenes":
                self.frames["StartPage"].log('Inicializando sistema...')
                self.update()
                try:
                    self.scan = Scan(int(self.dictionary["widthFrame"]), int(self.dictionary["heightFrame"]), 30, 10, 0)
                    self.scan.startPipeline()
                    resultado = prueba_1.Select_Imagenes_modo_2(50)
                    prueba_1.procesarDatos(resultado[0], resultado[1], resultado[2],2, lambda q1, numerototal: self.tomar_foto(q1, numerototal),)
                    self.frames["StartPage"].log('Se terminó de ejecutar la función de los datos')
                    self.frames["StartPage"].log('Esperando que se termine de evaluar la nube de puntos')
                    self.update()
                    self.scan.stopPipeline()
                    self.frames["StartPage"].log('Sistema listo para mostrar el modelo')
                    self.frames["StartPage"].Progress(360)
                    self.update()
                    self.enablePC = True
                except Exception as e: 
                    self.frames["StartPage"].log(f"Error al inicializar la cámara: {str(e)}")
                    self.scan = None  # Asegúrate de que la variable scan se restablezca si hay un error
                self.update()


            if selected_option == "70 imágenes":
                self.frames["StartPage"].log('Inicializando sistema...')
                self.update()
                try:
                    self.scan = Scan(int(self.dictionary["widthFrame"]), int(self.dictionary["heightFrame"]), 30, 10, 0)
                    self.scan.startPipeline()
                    resultado = prueba_1.Select_Imagenes_modo_2(70)
                    prueba_1.procesarDatos(resultado[0], resultado[1], resultado[2],2, lambda q1, numerototal: self.tomar_foto(q1, numerototal),)
                    self.frames["StartPage"].log('Se terminó de ejecutar la función de los datos')
                    self.frames["StartPage"].log('Esperando que se termine de evaluar la nube de puntos')
                    self.update()
                    self.scan.stopPipeline()
                    self.frames["StartPage"].log('Sistema listo para mostrar el modelo')
                    self.frames["StartPage"].Progress(360)
                    self.update()
                    self.enablePC = True
                except Exception as e: 
                    self.frames["StartPage"].log(f"Error al inicializar la cámara: {str(e)}")
                    self.scan = None  # Asegúrate de que la variable scan se restablezca si hay un error
                self.update()    
    def getCarpetaPath(self):
        return self.ruta_carpeta

    def showPC(self):     
        if self.scan is not None:
            o3d.visualization.draw_geometries([self.scan.getPointcloud()])
        else:
            self.frames["StartPage"].log("Error: La cámara no está inicializada o hubo un problema en el proceso de escaneo.")
        
    def makeSTL(self):
        if self.enablePC == True:
            self.STL = self.scan.makeSTL(int(self.dictionary["k_points"]), float(self.dictionary["std_ratio"]), int(self.dictionary["depth"]), int(self.dictionary["iterations"]))
            o3d.visualization.draw_geometries([self.STL])
            self.enableSaveSTL = True
            self.frames["StartPage"].saveButton.configure(bg="#f2f2f2")
            self.frames["StartPage"].log("STL creado")
        
    def saveSTL(self):
        if self.enableSaveSTL == True:
            self.directory = tk.filedialog.asksaveasfilename(initialfile="MySTL.stl")      
            o3d.io.write_triangle_mesh(self.directory, self.STL)
            self.frames["StartPage"].log(f"STL guardado en {self.directory}")
     
    def readSettings(self):
        file = open("settings.txt", "r")
        contents = file.read()
        dictionary = ast.literal_eval(contents)
        file.close()
        return dictionary
        
    def writeSettings(self, newdictionary):
        self.newdictionary = newdictionary
        with open("settings.txt", 'w') as self.data:
            self.data.write(str(self.newdictionary))
        
    def saveSettings(self):
        self.frames["SettingsPage"].enterSettings()
        self.writeSettings(self.dictionary)
        
    def close_windows(self):     
        self.destroy()
        
class StartPage(tk.Frame):
    
    def __init__(self, parent, controller,gestor_instance):
        tk.Frame.__init__(self, parent)
        self.gestor_instance = gestor_instance
        self.controller = controller

                # Load logo images and resize them
        self.logo_image = Image.open("Imagen_IPN.png")  # Replace with your image path
        self.logo_image_2 = Image.open("Imagen_upiiz.jpg")  # Replace with your image path
        self.logo_image = self.logo_image.resize((360, 190))  # Resize the image (width, height)
        self.logo_image_2 = self.logo_image_2.resize((90, 110))  # Resize the image (width, height)

        
     
        # Convert the images to PhotoImage
        self.logo_photo = ImageTk.PhotoImage(self.logo_image)
        self.logo_photo_2 = ImageTk.PhotoImage(self.logo_image_2)
        
        # Create a canvas to display the background image
        self.canvas = tk.Canvas(self, width=1050, height=1600)  # Adjust the size as needed
        self.canvas.grid(sticky="nw", row=0, column=0, columnspan=3, rowspan=10)
        
        # Place the logo images on the canvas
        self.canvas.create_image(0, 0, anchor='nw', image=self.logo_photo)  # Adjust position as needed
        self.canvas.create_image(950, 20, anchor='nw', image=self.logo_photo_2)  # Adjust position as needed
        
        # Keep a reference to avoid garbage collection
        self.canvas.image1 = self.logo_photo
        self.canvas.image2 = self.logo_photo_2

        self.buttonFrame = tk.Frame(self)


         # Botón para abrir una nueva ventana
        self.buttonNewWindow = tk.Button(self.buttonFrame, text='Gestor de Imagenes', width=20, height=2, command=self.gestor_instance.open_new_window, font=('Helvetica', 14), bg='gray')
        self.buttonNewWindow.grid(sticky="W", row=5, column=2, pady=5, padx=10)


        
        self.buttonStart = tk.Button(self.buttonFrame, text='Iniciar Captura', width=15, command=self.controller.startScan, font=('Helvetica', 14))
        self.buttonStart.grid(sticky="W", row=0, column=5, pady=5, padx=10)
        
        self.buttonShowPC = tk.Button(self.buttonFrame, text='Mostrar modelo', width=15, height=2, command=self.controller.showPC, font=('Helvetica', 14))
        self.buttonShowPC.grid(sticky="W", row=2, column=5, pady=5, padx=10)
             
        self.quitButton = tk.Button(self.buttonFrame, text='Salir', width=15, command=self.controller.close_windows, font=('Helvetica', 14))
        self.quitButton.grid(sticky="W", row=5, column=5, pady=5, padx=10)

        # Sliders
        label1 = tk.Label(self.buttonFrame, text="Posicion azimut", font=('Helvetica', 14))
        label1.grid(sticky="W", row=0, column=0, pady=5, padx=10)
        self.sliderAzimuth = tk.Scale(self.buttonFrame, from_=1, to=100, orient='horizontal', length=200)
        self.sliderAzimuth.grid(sticky="W", row=0, column=1, pady=5, padx=10)

        self.altitudeVar = tk.IntVar()
        label2 = tk.Label(self.buttonFrame, text="Posicion Altitud", font=('Helvetica', 14))
        label2.grid(sticky="W", row=1, column=0, pady=5, padx=10)
        self.sliderAltitude = tk.Scale(self.buttonFrame, from_=1, to=100, orient='horizontal', length=200,variable=self.altitudeVar, command=self.ensure_odd)
        self.sliderAltitude.grid(sticky="W", row=1, column=1, pady=5, padx=10)

        label3 = tk.Label(self.buttonFrame, text="Radio de la esfera", font=('Helvetica', 14))
        label3.grid(sticky="W", row=2, column=0, pady=5, padx=10)
        self.sliderRoll = tk.Scale(self.buttonFrame, from_=1, to=210, orient='horizontal', length=200)
        self.sliderRoll.grid(sticky="W", row=2, column=1, pady=5, padx=10)

        label4 = tk.Label(self.buttonFrame, text="Posicion Roll", font=('Helvetica', 14))
        label4.grid(sticky="W", row=3, column=0, pady=5, padx=10)
        self.sliderSphereRadius = tk.Scale(self.buttonFrame, from_=1, to=100, orient='horizontal', length=200)
        self.sliderSphereRadius.grid(sticky="W", row=3, column=1, pady=5, padx=10)

        self.selectLabel = tk.Label(self.buttonFrame, text="Seleccionar una opcion:", font=('Helvetica', 14))
        self.selectLabel.grid(sticky="W", row=0, column=4, pady=5, padx=10)

        self.options = ["27 imágenes", "50 imágenes", "70 imágenes"]
        self.selectedOption = tk.StringVar()
        self.selectMenu = tk.OptionMenu(self.buttonFrame, self.selectedOption, *self.options)
        self.selectMenu.grid(sticky="W", row=1, column=4, pady=5, padx=10)

        self.radioLabel = tk.Label(self.buttonFrame, text="Selecciona modo de captura:", font=('Helvetica', 14))
        self.radioLabel.grid(sticky="W", row=0, column=2, pady=5, padx=10)

        self.radioVar = tk.StringVar()
        self.radioVar.set("Modo 1")
        self.radio1 = tk.Radiobutton(self.buttonFrame, text="Modo 1", variable=self.radioVar, value="Option 1", font=('Helvetica', 14))
        self.radio1.grid(sticky="W", row=1, column=2, pady=5, padx=10)

        self.radio2 = tk.Radiobutton(self.buttonFrame, text="Modo 2", variable=self.radioVar, value="Option 2", font=('Helvetica', 14))
        self.radio2.grid(sticky="W", row=2, column=2, pady=5, padx=10)

        self.buttonFrame.grid(sticky="W", row=0, column=1)
        
        self.load = Image.fromarray(np.zeros(shape=(480, 848, 3)), 'RGB')
        self.left, self.upper, self.right, self.lower = 330, 20, 848, 480

        self.render = ImageTk.PhotoImage(self.load.crop((self.left, self.upper, self.right, self.lower)))
        self.canvas_image = tk.Canvas(self, width=self.right-self.left, height=self.lower-self.upper)  
        self.canvas_image.grid(sticky="s", row=0, column=2)
        self.canvas_image.create_image(0, 0, anchor='nw', image=self.render)    
        self.canvas_image.image = self.render  

        self.text_area = tk.Text(self, height=10, wrap='word')
        self.text_area.grid(sticky="we", row=1, column=0, columnspan=3, pady=10, padx=10)

   




    def ensure_odd(self, value):
        value = int(value)
        if value % 2 == 0:
            value += 1
        self.altitudeVar.set(value)  

    def log(self, message):
        self.text_area.insert(tk.END, message + '\n')
        self.text_area.see(tk.END)

    def getSliderData(self):
        azimuth = self.sliderAzimuth.get()
        altitude = self.sliderAltitude.get()
        roll = self.sliderRoll.get()
        new_roll = self.sliderSphereRadius.get()
        selected_option = self.selectedOption.get()
        radio_var = self.radioVar.get()
        
        return azimuth, altitude, roll, new_roll, selected_option, radio_var

    def showImage(self, iArray):
        self.load = Image.fromarray(iArray, 'RGB')
        self.render = ImageTk.PhotoImage(self.load.crop((self.left, self.upper, self.right, self.lower)))
        self.canvas_image.create_image(0, 0, anchor='nw', image=self.render)    
        self.canvas_image.image = self.render
    
    def startProgress(self):
        self.progress = Progressbar(self, orient="horizontal", length=self.right-self.left, mode='determinate', maximum=360)
        self.progress.grid(sticky="NW", row=1, column=1, pady=5) 
    
    def Progress(self, getal):
        self.progress['value'] = getal
    
    def endProgress(self):
        self.progress.grid_forget()  




class SettingsEntry:
    
    def __init__(self, master, name, **kwargs):
        self.name = name
        self.master = master
        self.var = kwargs.get('var', None)
        self.frame = tk.Frame(self.master)
        self.label = tk.Label(self.frame, text=self.name, width=15, anchor='e')
        self.label.pack(side="left", fill="both")
        self.entry = tk.Entry(self.frame)
        self.entry.pack(side="left", fill="both")
        self.entry.delete(0, 'end')
        self.entry.insert(0, self.var)
        self.frame.pack()
        
    def get(self):       
        return self.entry.get()
    
    def insert(self, newVar):
        self.entry.delete(0, 'end')
        self.entry.insert(0, newVar)

        
class SettingsPage(tk.Frame):
    
    def __init__(self, parent, controller,gestor_instance):
        tk.Frame.__init__(self, parent)
        
        self.controller = controller
        self.gestor_instance = gestor_instance  # Store the gestor_instance
        
        self.buttonFrame = tk.Frame(self)
        self.buttonMp = tk.Button(self.buttonFrame, text = 'Ventana principal', width = 25, command=lambda: controller.show_frame("StartPage"),font=('Helvetica', 14))
        self.buttonMp.grid(sticky="W",row = 0, column = 0, pady = 5, padx = 10)
        
        # self.buttonEntersettings = tk.Button(self.buttonFrame, text="Use Settings",width = 25,command=self.enterSettings)
        # self.buttonEntersettings.grid(sticky="W",row = 1, column = 0, pady = 5, padx = 10)
        
        self.buttonDefaultsettings = tk.Button(self.buttonFrame, text="Reiniciar la configuracion",width = 25,command=self.defaultSettings)
        self.buttonDefaultsettings.grid(sticky="W",row = 1, column = 0, pady = 5, padx = 10)
        
        self.buttonSavesettings = tk.Button(self.buttonFrame, text="Guardar la configuracion",width = 25,command=self.controller.saveSettings)
        self.buttonSavesettings.grid(sticky="W",row = 2, column = 0, pady = 5, padx = 10)
        
        self.quitButton = tk.Button(self.buttonFrame, text = 'Salir', width = 25, command = self.controller.close_windows)
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
    