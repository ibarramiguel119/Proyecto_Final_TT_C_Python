import tkinter as tk
from tkinter import filedialog
from tkinter import messagebox
from PIL import Image, ImageTk
import sqlite3
import os
import cv2
import time  # Para crear una pausa entre capturas

# Conectar a la base de datos SQLite (crear si no existe)
def crear_bd():
    conn = sqlite3.connect('imagenes.db')
    cursor = conn.cursor()
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS fotos (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            nombre TEXT NOT NULL,
            ruta TEXT NOT NULL
        )
    ''')
    conn.commit()
    conn.close()

# Función para capturar 10 imágenes secuenciales desde la webcam y guardarlas en una carpeta
def capturar_imagenes_secuenciales():
    cap = cv2.VideoCapture(0)  # Abrir la webcam
    if not cap.isOpened():
        messagebox.showerror("Error", "No se pudo acceder a la webcam.")
        return

    # Crear una carpeta con un nombre basado en la hora actual para almacenar las imágenes
    nombre_carpeta = time.strftime("capturas_%Y%m%d_%H%M%S")
    os.makedirs(nombre_carpeta, exist_ok=True)
    
    # Capturar 10 imágenes secuenciales
    for i in range(10):
        ret, frame = cap.read()  # Capturar una imagen
        if ret:
            # Guardar cada imagen capturada en la carpeta
            ruta = os.path.join(nombre_carpeta, f"captura_{i+1}.jpg")
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

    messagebox.showinfo("Éxito", f"10 imágenes capturadas y guardadas en la carpeta {nombre_carpeta}.")

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
            try:
                img = Image.open(ruta)
                img.thumbnail((100, 100))  # Redimensionar para vista previa
                img_tk = ImageTk.PhotoImage(img)
                label = tk.Label(frame_imagenes, image=img_tk, text=nombre, compound=tk.TOP)
                label.image = img_tk  # Mantener la referencia
                label.grid(row=idx//5, column=idx%5, padx=5, pady=5)
            except Exception as e:
                print(f"No se pudo cargar la imagen {nombre}: {e}")
    
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

# Crear la interfaz gráfica con Tkinter
root = tk.Tk()
root.title("Gestor de Imágenes")
root.geometry("600x400")

# Frame para los botones
frame_botones = tk.Frame(root)
frame_botones.pack(pady=10)

# Botones para capturar, mostrar y eliminar imágenes
boton_capturar = tk.Button(frame_botones, text="Capturar 10 Imágenes con Webcam", command=capturar_imagenes_secuenciales)
boton_capturar.grid(row=0, column=0, padx=5)

boton_mostrar = tk.Button(frame_botones, text="Mostrar Imágenes", command=mostrar_imagenes)
boton_mostrar.grid(row=0, column=1, padx=5)

boton_eliminar = tk.Button(frame_botones, text="Eliminar Imagen", command=eliminar_imagen)
boton_eliminar.grid(row=0, column=2, padx=5)

# Frame para mostrar las imágenes
frame_imagenes = tk.Frame(root)
frame_imagenes.pack(pady=20)

# Crear la base de datos si no existe
crear_bd()

root.mainloop()
