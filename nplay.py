import cairo
import pygame
import math
import os.path

from nsim import *

def hex2float(string):
    value = int(string, 16)
    red = ((value & 0xFF0000) >> 16) / 255
    green = ((value & 0x00FF00) >> 8) / 255
    blue = (value & 0x0000FF) / 255
    return red, green, blue

width = 1056
height = 600

pygame.init()
pygame.display.set_caption("N++")
screen = pygame.display.set_mode((width, height))
clock = pygame.time.Clock()
running = True

surface = cairo.ImageSurface(cairo.Format.RGB24, width, height)
ctx = cairo.Context(surface)

sim = Simulator()
with open("map_data", "rb") as f:
    sim.load([int(b) for b in f.read()])

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    
    keys = pygame.key.get_pressed()
    hor_input = 0
    jump_input = 0
    if keys[pygame.K_RIGHT]:
        hor_input = 1
    if keys[pygame.K_LEFT]:
        hor_input = -1
    if keys[pygame.K_z]:
        jump_input = 1

    ctx.set_source_rgb(*hex2float("cbcad0"))
    ctx.rectangle(0, 0, width, height)
    ctx.fill()

    ctx.set_source_rgb(*hex2float("797988"))
    ctx.set_line_width(2)
    for cell in sim.segment_dic.values():
        for segment in cell:
            if segment.type == "linear":
                ctx.move_to(segment.x1, segment.y1)
                ctx.line_to(segment.x2, segment.y2)
            elif segment.type == "circular":
                angle = math.atan2(segment.hor, segment.ver) + (math.pi if segment.hor != segment.ver else 0)
                ctx.arc(segment.xpos, segment.ypos, segment.radius, angle - math.pi/4, angle + math.pi/4)
        ctx.stroke()

    ctx.set_source_rgb(*hex2float("000000"))
    ctx.arc(sim.ninja.xpos, sim.ninja.ypos, sim.ninja.RADIUS, 0, 2 * math.pi)
    ctx.fill()

    sim.tick(hor_input, jump_input)

    buffer = surface.get_data()
    image = pygame.image.frombuffer(buffer, (width, height), "BGRA")
    screen.blit(image, (0, 0))

    pygame.display.flip()
    clock.tick(60)

pygame.quit()