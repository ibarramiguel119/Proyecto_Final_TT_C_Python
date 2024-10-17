import pyrealsense2 as rs
import numpy as np
import math
import time

# Función para convertir de radianes a grados
def rad_to_deg(radians):
    return radians * 180 / math.pi

# Función para integrar la velocidad angular si el cambio es significativo
def integrate_angular_velocity(angular_velocity, orientation, dt, threshold=0.01):
    # Solo actualizamos la orientación si la velocidad angular es mayor que el umbral
    if np.abs(angular_velocity).max() > threshold:
        orientation += angular_velocity * dt
    return orientation

def main():
    # Inicializar pipeline y configurar los streams para giroscopio y acelerómetro
    imu_pipeline = rs.pipeline()
    imu_config = rs.config()
    
    imu_config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 63)  # aceleración
    imu_config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)  # giroscopio
    imu_profile = imu_pipeline.start(imu_config)

    # Inicializar variables para posiciones angulares (roll, pitch, yaw)
    orientation = np.array([0.0, 0.0, 0.0], dtype=np.float64)  # Roll, Pitch, Yaw

    # Tiempo inicial
    last_time = time.time()

    try:
        print("Streaming IMU data...")

        while True:
            # Obtener los frames
            frames = imu_pipeline.wait_for_frames()

            # Obtener los datos del giroscopio
            gyro_frame = frames.first_or_default(rs.stream.gyro)
            accel_frame = frames.first_or_default(rs.stream.accel)

            if gyro_frame:
                gyro_data = gyro_frame.as_motion_frame().get_motion_data()

                # Calcular el tiempo entre frames
                current_time = time.time()
                dt = current_time - last_time  # delta time (tiempo entre mediciones)
                last_time = current_time

                # Convertir los datos de giroscopio a un arreglo numpy
                angular_velocity = np.array([gyro_data.x, gyro_data.y, gyro_data.z], dtype=np.float64)

                # Integrar la velocidad angular para obtener la orientación
                orientation = integrate_angular_velocity(angular_velocity, orientation, dt)

                # Convertir a grados para imprimir
                roll_deg = rad_to_deg(orientation[0])
                pitch_deg = rad_to_deg(orientation[1])
                yaw_deg = rad_to_deg(orientation[2])

                print(f"Gyro -> Roll: {roll_deg:.2f}°, Pitch: {pitch_deg:.2f}°, Yaw: {yaw_deg:.2f}°")

            if accel_frame:
                accel_data = accel_frame.as_motion_frame().get_motion_data()
                print(f"Accel -> x: {accel_data.x:.4f}, y: {accel_data.y:.4f}, z: {accel_data.z:.4f}")

    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        # Detener el pipeline
        try:
            imu_pipeline.stop()
            print("Pipeline stopped successfully.")
        except Exception as e:
            print(f"Error stopping the pipeline: {e}")

if __name__ == "__main__":
    main()
