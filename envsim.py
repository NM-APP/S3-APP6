import pygame
import subprocess
import mmap
import ctypes
import os
from posix_ipc import Semaphore, O_CREAT
import random
import math
import time

MAX_ROBOTS = 100000

class RobotState(ctypes.Structure):
    _fields_ = [("x", ctypes.c_double),
                ("y", ctypes.c_double),
                ("t", ctypes.c_double),
                ("dx", ctypes.c_double),
                ("dt", ctypes.c_double),
                ("ux", ctypes.c_double),
                ("ut", ctypes.c_double)]

class SharedMemory(ctypes.Structure):
    _fields_ = [("robots", RobotState * MAX_ROBOTS),
                ("message", ctypes.c_int32)]

MSG_NONE = 0
MSG_STOP = 1

WIDTH, HEIGHT = 800, 800
ROBOT_SIZE = 5
ROBOT_COUNT = 5000

def main():
    sem_start_name = f"gro300_sem_start_{random.randint(1000,9999)}"
    sem_done_name = f"gro300_sem_done_{random.randint(1000,9999)}"

    # Create and initialize semaphore
    sem_start = Semaphore(sem_start_name, O_CREAT, initial_value=0)
    sem_done = Semaphore(sem_done_name, O_CREAT, initial_value=0)

    process = subprocess.Popen(["./robotsim",
                               sem_start_name,
                               sem_done_name,
                               str(ROBOT_COUNT)], 
                               stdout=subprocess.PIPE) 
                               #stderr=subprocess.PIPE)

    # Read shared memory filename from robotsim's output
    shared_mem_fname = process.stdout.readline().decode().strip()

    if (shared_mem_fname == ""):
        print("Erreur: Impossible de lire le nom du fichier de mémoire partagée.")
        return

    # Open the shared memory file and create a memory map
    with open(shared_mem_fname, "r+b") as f:
        mm = mmap.mmap(f.fileno(), 0)

    shared_data = SharedMemory.from_buffer(mm)

    running = True
    cycle_count = 0
    time_spent_ms = 0

    print("PID du processus robotsim:", process.pid)
    input("Appuyez sur Entrée pour lancer la simulation...")

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Simulation de Robots")
    clock = pygame.time.Clock()

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        start_time = time.perf_counter()

        # Signal robotsim to perform a step
        sem_start.release()

        # Wait for the step to complete
        sem_done.acquire()

        end_time = time.perf_counter()
        time_spent_ms += (end_time - start_time) * 1000

        # Draw robots
        screen.fill((255, 255, 255))
        for robot in shared_data.robots[:ROBOT_COUNT]:
            x = int(robot.x * WIDTH)
            y = int((1 - robot.y) * HEIGHT)
            angle = robot.t
            points = [
                (x + ROBOT_SIZE * math.cos(angle), y - ROBOT_SIZE * math.sin(angle)),
                (x + ROBOT_SIZE * math.cos(angle + 2.6), y - ROBOT_SIZE * math.sin(angle + 2.6)),
                (x + ROBOT_SIZE * math.cos(angle - 2.6), y - ROBOT_SIZE * math.sin(angle - 2.6))
            ]
            pygame.draw.polygon(screen, (0, 0, 255), points)
        
        pygame.display.flip()

        cycle_count += 1
        clock.tick(60)

    print(f"Cycle count: {cycle_count}, average time per cycle: {time_spent_ms / cycle_count:.2f} ms")

    pygame.quit()

    shared_data.message = MSG_STOP
    sem_start.release()
    process.wait()

    # NOTE: The subprocess has the responsibility to unlink the shared memory file
    # Everything should be cleaned after the process is done.

    sem_start.unlink()
    sem_done.unlink()

if __name__ == "__main__":
    main()