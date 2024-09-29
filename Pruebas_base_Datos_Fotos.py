import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from PIL import Image, ImageTk
import sqlite3
import os
import cv2
import time

# Conectar a la base de datos SQLite (crear si no existe)
def crear_bd():
    conn = sqlite3.connect('imagenes.db')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE IF NOT EXISTS fotos (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        nombre TEXT NOT NULL,
        ruta TEXT NOT NULL
    )''')
    conn.commit()
    conn.close()

# Función para capturar 10 imágenes secuenciales desde la webcam y guardarlas en una carpeta
def capturar_imagenes_secuenciales():
    cap = cv2.VideoCapture(0)  # Abrir la webcam
    if not cap.isOpened():
        messagebox.showerror("Error", "No se pudo acceder a la webcam.")
        return

    # Crear la carpeta 'Objetos_Capturados' si no existe
    carpeta_principal = "Objetos_Capturados"
    os.makedirs(carpeta_principal, exist_ok=True)

    # Crear una subcarpeta con un nombre basado en la hora actual para almacenar las imágenes
    nombre_carpeta = time.strftime("Objeto_%Y%m%d_%H%M%S")
    ruta_carpeta = os.path.join(carpeta_principal, nombre_carpeta)
    os.makedirs(ruta_carpeta, exist_ok=True)
    
    # Capturar 10 imágenes secuenciales
    for i in range(10):
        ret, frame = cap.read()  # Capturar una imagen
        if ret:
            # Guardar cada imagen capturada en la carpeta
            ruta = os.path.join(ruta_carpeta, f"Imagen_{i+1}.jpg")
            cv2.imwrite(ruta, frame)
            
            # Agregar la imagen capturada a la base de datos
            nombre = os.path.basename(ruta)
            conn = sqlite3.connect('imagenes.db')
            cursor = conn.cursor()
            cursor.execute("INSERT INTO fotos (nombre, ruta) VALUES (?, ?)", (nombre, ruta))
            conn.commit()
            conn.close()

            time.sleep(0.5)  # Pausa de 0.5 segundos entre capturas
        else:
            messagebox.showerror("Error", "No se pudo capturar la imagen.")
            break

    cap.release()
    cv2.destroyAllWindows()
    messagebox.showinfo("Éxito", f"10 imágenes capturadas y guardadas en la carpeta {ruta_carpeta}.")
    crear_arbol_directorios()  # Actualizar el árbol después de capturar imágenes

# Función para listar carpetas y mostrarlas
def listar_carpetas():
    for widget in frame_carpetas.winfo_children():
        widget.destroy()  # Limpiar el frame antes de mostrar las nuevas carpetas

    carpetas = [d for d in os.listdir("Objetos_Capturados") if os.path.isdir(os.path.join("Objetos_Capturados", d))]
    
    if carpetas:
        for idx, carpeta in enumerate(carpetas):
            label = tk.Label(frame_carpetas, text=carpeta, anchor="w")
            label.grid(row=idx, column=0, padx=5, pady=2, sticky="w")
    else:
        label = tk.Label(frame_carpetas, text="No se encontraron carpetas de capturas.", anchor="w")
        label.grid(row=0, column=0, padx=5, pady=2)

# Función para mostrar las imágenes almacenadas en la base de datos
def mostrar_imagenes():
    for widget in frame_imagenes.winfo_children():
        widget.destroy()  # Limpiar el frame antes de mostrar las nuevas imágenes
    
    conn = sqlite3.connect('imagenes.db')
    cursor = conn.cursor()
    cursor.execute("SELECT nombre, ruta FROM fotos")
    imagenes = cursor.fetchall()
    
    if imagenes:
        for idx, (nombre, ruta) in enumerate(imagenes):
            if os.path.exists(ruta):  # Verifica si el archivo existe
                try:
                    img = Image.open(ruta)
                    img.thumbnail((100, 100))  # Redimensionar para vista previa
                    img_tk = ImageTk.PhotoImage(img)
                    label = tk.Label(frame_imagenes, image=img_tk, text=nombre, compound=tk.TOP)
                    label.image = img_tk  # Mantener la referencia
                    label.grid(row=idx//5, column=idx%5, padx=5, pady=5)
                except Exception as e:
                    print(f"No se pudo cargar la imagen {nombre}: {e}")
            else:
                print(f"No se encontró la imagen en la ruta: {ruta}")
    
    conn.close()

# Función para eliminar una imagen de la base de datos
def eliminar_imagen():
    ruta = filedialog.askopenfilename(title="Seleccionar imagen para eliminar", filetypes=[("Archivos de imagen", "*.jpg;*.png")])
    if ruta:
        nombre = os.path.basename(ruta)
        conn = sqlite3.connect('imagenes.db')
        cursor = conn.cursor()
        cursor.execute("DELETE FROM fotos WHERE nombre=? AND ruta=?", (nombre, ruta))
        conn.commit()
        conn.close()
        messagebox.showinfo("Éxito", f"Imagen {nombre} eliminada de la base de datos.")

# Función para crear el árbol de directorios
def crear_arbol_directorios():
    # Obtener la lista de directorios y archivos en la ruta actual
    path = os.path.join(os.getcwd(), "Objetos_Capturados")
    for widget in frame_directorios.winfo_children():
        widget.destroy()  # Limpiar el frame antes de mostrar los nuevos elementos
    
    tree = ttk.Treeview(frame_directorios)
    tree.pack(fill=tk.BOTH, expand=True)
    
    # Llenar el árbol con los directorios y archivos
    def agregar_elementos_arbol(parent, ruta):
        for item in os.listdir(ruta):
            item_path = os.path.join(ruta, item)
            oid = tree.insert(parent, 'end', text=item, open=False)
            if os.path.isdir(item_path):
                agregar_elementos_arbol(oid, item_path)
    
    # Insertar el directorio raíz
    root_id = tree.insert('', 'end', text=path, open=True)
    agregar_elementos_arbol(root_id, path)

# Crear la interfaz gráfica con Tkinter
root = tk.Tk()
root.title("Gestor de Imágenes")
root.geometry("1100x600")

# Frame para mostrar el árbol de directorios
frame_directorios = tk.Frame(root)
frame_directorios.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")

# Frame para mostrar las imágenes
frame_imagenes = tk.Frame(root)
frame_imagenes.grid(row=1, column=1, padx=10, pady=10, sticky="nsew")

# Frame para los botones
frame_botones = tk.Frame(root)
frame_botones.grid(row=1, column=2, padx=10, pady=10, sticky="ns")  # Cambiar a columna 2

# Estilo para los botones
boton_estilo = {
    'width': 25,         # Ancho del botón
    'height': 2,        # Altura del botón
    'font': ('Arial', 12),  # Fuente y tamaño de la letra
}

# Botones para capturar, mostrar y eliminar imágenes
boton_capturar = tk.Button(frame_botones, text="Capturar Imagenes", command=capturar_imagenes_secuenciales, **boton_estilo)
boton_capturar.pack(padx=10, pady=100)

boton_mostrar = tk.Button(frame_botones, text="Mostrar Imágenes", command=mostrar_imagenes, **boton_estilo)
boton_mostrar.pack(padx=10, pady=100)

#boton_eliminar = tk.Button(frame_botones, text="Eliminar Imagen", command=eliminar_imagen, **boton_estilo)
#boton_eliminar.pack(padx=10, pady=100)

# Crear la base de datos si no existe
crear_bd()

# Llamar a crear_arbol_directorios al iniciar
crear_arbol_directorios()

# Configurar el peso para expandir las columnas
root.grid_columnconfigure(0, weight=1)
root.grid_columnconfigure(1, weight=3)  # Aumentar peso para la columna de imágenes
root.grid_columnconfigure(2, weight=0)  # La columna de botones no necesita ser expandible
root.grid_rowconfigure(1, weight=1)  # Aumentar peso para la fila de imágenes

root.mainloop()
