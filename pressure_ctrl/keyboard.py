import pygame
import sys
import socket
from typing import Tuple


"""
Press "0-9" keys to quickly change the pressure, use "w/s" keys to fine tune
Press "b" key to apply the brake
"""


# send to this address
UDP_IP = "10.253.162.49"  # bumblebee
UDP_PORT = 5005

# step size of pressure change by pressing w/s keys
_CONST_STEP_SIZE = 0.2
# number of intervals to wait to execute above step
_CNT_EXEC = 5

# for 0 - 9 keys
# e.g. 1: 10% of below
# e.g. 7: 70% of below
# NOTE: 0: 100% of below
_MAX_PRESSURE_BARS = 16


class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 25)

    def tprint(self, screen, text):
        text_bitmap = self.font.render(text, True, (0, 0, 0))
        screen.blit(text_bitmap, (self.x, self.y))
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10


def _main_loop(
    sock: socket.socket,
    udp_addr: Tuple[str, int],
    interval_ms: int,
):
    pygame.init()
    display = pygame.display.set_mode((500, 500))
    pygame.display.set_caption('set pressure')

    # pygame screen text
    text_print = TextPrint()

    kmap = {
        1: pygame.K_1,
        2: pygame.K_2,
        3: pygame.K_3,
        4: pygame.K_4,
        5: pygame.K_5,
        6: pygame.K_6,
        7: pygame.K_7,
        8: pygame.K_8,
        9: pygame.K_9,
        10: pygame.K_0,
    }

    output = 0
    setpoint = 0
    cnt_inc = 0
    cnt_dec = 0

    while True:
        # for showing the current keys
        display.fill((255, 255, 255))
        text_print.reset()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            
        pressed = pygame.key.get_pressed()


        for k, val in kmap.items():
            if pressed[val]:
                # print("key 0")
                perc = k * 10
                bars = perc * _MAX_PRESSURE_BARS / 100.0
                setpoint = round(setpoint, 1)
                print(f"{perc}% = {bars} bars")
                setpoint = bars

        if pressed[pygame.K_w]:
            cnt_inc += 1
            if cnt_inc >= _CNT_EXEC:
                setpoint += _CONST_STEP_SIZE
                setpoint = round(setpoint, 1)
                print(f"+, {setpoint} bars")          
                cnt_inc = 0
        if pressed[pygame.K_s]:
            cnt_dec += 1
            if cnt_dec >= _CNT_EXEC:
                if setpoint > _CONST_STEP_SIZE:
                    setpoint -= _CONST_STEP_SIZE
                    setpoint = round(setpoint, 1)
                    print(f"-, {setpoint} bars")          
                else:
                    print("setpoint cannot be lower than 0 bar")
                cnt_dec = 0

        setpoint = round(setpoint, 1)
        if pressed[pygame.K_b]:
            output = setpoint
            print(f"braking with {output} bar") 
            # SEND THE OUTPUT TO UDP SOCKET HERE
            display.fill((255, 0, 0))
        else:
            output = 0

        sock.sendto(str(output).encode(), udp_addr)

        msg1 = f"Internal: {setpoint} bar(s)"
        msg2 = f"Final: {output} bar(s)"
        # display in window
        text_print.tprint(display, msg1)
        text_print.tprint(display, msg2)

        pygame.display.flip()
        pygame.time.wait(interval_ms)


def _main():
    # socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    _main_loop(
        sock=sock,
        udp_addr=(UDP_IP, UDP_PORT),
        interval_ms=20,
    )


if __name__ == "__main__":
    _main()
