from controller import Robot
import random
import math
import matplotlib.pyplot as plt
import numpy as np

# Crear instancia del robot
TIME_STEP = 32
robot = Robot()

# Obtener dispositivos
left_motor = robot.getDevice('left_motor')
right_motor = robot.getDevice('right_motor')
lidar = robot.getDevice('lidar')

# Configurar motores para velocidad infinita
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# Velocidades iniciales
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Habilitar LIDAR
lidar.enable(TIME_STEP)
lidar.enablePointCloud()

# Obtener parámetros del LIDAR
resolution = lidar.getHorizontalResolution()
fov = lidar.getFov()
max_range = lidar.getMaxRange()

# Calcular ángulos por rayo
angles = np.linspace(-fov / 2, fov / 2, resolution)

# Variables para movimiento aleatorio
change_interval = 1500  # Cambiar dirección cada 1.5 segundos
counter = 0
max_speed = 8.0  # Velocidad aumentada

# Configurar matplotlib para visualización en tiempo real
plt.ion()
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

# Subplot 1: Escaneo actual del LIDAR
scan_plot, = ax1.plot([], [], 'g.', markersize=3, label='Escaneo actual')
robot_plot, = ax1.plot([0], [0], 'ro', markersize=8, label='Robot')
ax1.set_xlim(-3, 3)
ax1.set_ylim(-3, 3)
ax1.set_aspect('equal')
ax1.set_title("LIDAR - Vista en Tiempo Real", fontsize=12, fontweight='bold')
ax1.set_xlabel("X (m)")
ax1.set_ylabel("Y (m)")
ax1.grid(True, alpha=0.3)
ax1.legend(loc='upper right')

# Subplot 2: Mapa acumulado
map_xs = []
map_ys = []
map_plot, = ax2.plot([], [], 'b.', markersize=1, alpha=0.6, label='Mapa acumulado')
robot_map_plot, = ax2.plot([0], [0], 'ro', markersize=8, label='Robot')
ax2.set_xlim(-2, 2)
ax2.set_ylim(-2, 2)
ax2.set_aspect('equal')
ax2.set_title("Mapa 2D Acumulado", fontsize=12, fontweight='bold')
ax2.set_xlabel("X (m)")
ax2.set_ylabel("Y (m)")
ax2.grid(True, alpha=0.3)
ax2.legend(loc='upper right')

plt.tight_layout()

print("Robot iniciado - Movimiento aleatorio con LIDAR y mapeo 2D en tiempo real")
print(f"Velocidad máxima: {max_speed} rad/s")
print(f"Resolución LIDAR: {resolution} puntos")
print(f"Campo de visión: {math.degrees(fov):.1f}°")

# Loop principal
while robot.step(TIME_STEP) != -1:
    counter += TIME_STEP
    
    # Cambiar dirección aleatoriamente
    if counter >= change_interval:
        counter = 0
        
        # Generar velocidades aleatorias con más variación
        movement_type = random.choice(['forward', 'forward', 'turn', 'curve'])
        
        if movement_type == 'forward':
            # Movimiento hacia adelante rápido
            left_speed = random.uniform(max_speed * 0.7, max_speed)
            right_speed = left_speed + random.uniform(-1, 1)
        elif movement_type == 'turn':
            # Giro en el lugar
            turn_speed = random.uniform(max_speed * 0.5, max_speed)
            left_speed = turn_speed if random.random() > 0.5 else -turn_speed
            right_speed = -left_speed
        else:
            # Curva suave
            base_speed = random.uniform(max_speed * 0.5, max_speed * 0.8)
            left_speed = base_speed
            right_speed = base_speed * random.uniform(0.3, 0.9)
        
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        
        print(f"Nueva dirección [{movement_type}] - Izq: {left_speed:.2f}, Der: {right_speed:.2f}")
    
    # Obtener datos del LIDAR
    distances = lidar.getRangeImage()
    
    if distances:
        # Convertir a coordenadas cartesianas (x, y) para el escaneo actual
        xs = []
        ys = []
        
        for r, a in zip(distances, angles):
            if 0.01 < r < max_range:
                x = r * math.cos(a)
                y = r * math.sin(a)
                xs.append(x)
                ys.append(y)
                
                # Agregar al mapa acumulado
                map_xs.append(x)
                map_ys.append(y)
        
        # Actualizar gráfico del escaneo actual
        scan_plot.set_xdata(xs)
        scan_plot.set_ydata(ys)
        
        # Actualizar mapa acumulado
        map_plot.set_xdata(map_xs)
        map_plot.set_ydata(map_ys)
        
        # Actualizar título con información
        ax2.set_title(f"Mapa 2D Acumulado ({len(map_xs)} puntos)", 
                     fontsize=12, fontweight='bold')
        
        plt.draw()
        plt.pause(0.001)
        
        # Detectar obstáculos cercanos
        min_distance = min(distances)
        
        if min_distance < 0.25:
            print(f"¡Obstáculo detectado a {min_distance:.2f}m! - Evadiendo...")
            left_motor.setVelocity(-max_speed * 0.7)
            right_motor.setVelocity(max_speed * 0.9)
            counter = -800  # Dar tiempo para maniobrar