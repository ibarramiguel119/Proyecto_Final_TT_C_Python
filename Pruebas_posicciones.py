import pyrealsense2 as rs
import numpy as np
import math
import time

# Función para convertir un vector a grados
def rad_to_deg(radians):
    return radians * 180 / math.pi

# Función para integrar la aceleración para obtener la posición
def integrate_acceleration(accel_data, dt):
    # Convertir los datos de aceleración a un arreglo de numpy y ajustar la aceleración para que sea en m/s^2
    accel_data = np.array([accel_data.x, accel_data.y, accel_data.z], dtype=np.float64) * 9.81
    
    # Integrar la aceleración para obtener la velocidad
    velocity = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    velocity += accel_data * dt
    
    # Integrar la velocidad para obtener la posición
    position = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    position += velocity * dt
    
    return position

# Función para integrar la velocidad angular para obtener la orientación
def integrate_angular_velocity(angular_velocity, dt):
    # Integrar la velocidad angular para obtener la orientación
    orientation = np.array([0.0, 0.0, 0.0], dtype=np.float64)
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
    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    # Inicializar variables para posición y velocidad como float64
    position = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    velocity = np.array([0.0, 0.0, 0.0], dtype=np.float64)

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

                # Integrar las velocidades angulares del giroscopio para obtener ángulos
                roll += gyro_data.x * dt  # Rotación sobre el eje X
                pitch += gyro_data.y * dt  # Rotación sobre el eje Y
                yaw += gyro_data.z * dt  # Rotación sobre el eje Z

                # Convertir a grados para imprimir
                roll_deg = rad_to_deg(roll)
                pitch_deg = rad_to_deg(pitch)
                yaw_deg = rad_to_deg(yaw)

                print(f"Gyro -> Roll: {roll_deg:.2f}°, Pitch: {pitch_deg:.2f}°, Yaw: {yaw_deg:.2f}°")

            if accel_frame:
                accel_data = accel_frame.as_motion_frame().get_motion_data()
                print(f"Accel -> x: {accel_data.x:.4f}, y: {accel_data.y:.4f}, z: {accel_data.z:.4f}")

                # Integrar la aceleración para obtener la posición y velocidad
                position += integrate_acceleration(accel_data, dt)
                velocity += np.array([accel_data.x, accel_data.y, accel_data.z], dtype=np.float64) * 9.81 * dt  # Para actualizar la velocidad correctamente

                # Integrar la velocidad angular para obtener la orientación
                angular_velocity = np.array([gyro_data.x, gyro_data.y, gyro_data.z], dtype=np.float64)
                orientation = integrate_angular_velocity(angular_velocity, dt)

                # Convertir la orientación a grados para imprimir
                roll_deg = rad_to_deg(orientation[0])
                pitch_deg = rad_to_deg(orientation[1])
                yaw_deg = rad_to_deg(orientation[2])

                #print(f"Posición -> X: {position[0]:.2f}, Y: {position[1]:.2f}, Z: {position[2]:.2f}")
                #print(f"Orientación -> Roll: {roll_deg:.2f}°, Pitch: {pitch_deg:.2f}°, Yaw: {yaw_deg:.2f}°")

                

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
