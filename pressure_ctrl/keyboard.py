import pygame
import sys

UDP_IP = "10.253.162.52"
UDP_PORT = 5005

pygame.init()
display = pygame.display.set_mode((300,300))
pygame.display.set_caption('set pressure')

output = 0
set = 0

while True:
    pygame.time.wait(100)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                set += 1
                print(f"setpoint increased to {set} bar")          
            if event.key == pygame.K_s:
                if set != 0:
                    set -= 1
                    print(f"setpoint decreased to {set} bar")
                else:
                    print(f"setpoint cannot be lower than 0 bar ")
        
        pressed = pygame.key.get_pressed()
        if pressed[pygame.K_b]:
            output = set
            print(f"braking with {output} bar") 
            # SEND THE OUTPUT TO UDP SOCKET HERE
            output = 0



