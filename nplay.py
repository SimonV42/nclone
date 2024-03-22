import cairo
import pygame
import math
import os.path
import struct

from nsim import *

SRCWIDTH = 1056
SRCHEIGHT = 600

BGCOLOR = "cbcad0"
TILECOLOR = "797988"
NINJACOLOR = "000000"
ENTITYCOLOR = "882276"

SEGMENTWIDTH = 1
NINJAWIDTH = 1.25
DOORWIDTH = 2
PLATFORMWIDTH = 3

LIMBS = ((0, 12), (1, 12), (2, 8), (3, 9), (4, 10), (5, 11), (6, 7), (8, 0), (9, 0), (10, 1), (11, 1))

def hex2float(string):
    value = int(string, 16)
    red = ((value & 0xFF0000) >> 16) / 255
    green = ((value & 0x00FF00) >> 8) / 255
    blue = (value & 0x0000FF) / 255
    return red, green, blue

pygame.init()
pygame.display.set_caption("N++")
screen = pygame.display.set_mode((SRCWIDTH, SRCHEIGHT), pygame.RESIZABLE)
clock = pygame.time.Clock()
running = True

sim = Simulator()
with open("map_data", "rb") as f:
    mapdata = [int(b) for b in f.read()]
sim.load(mapdata)

def tiledraw(init):
    if init:
        tiledraw.surface = cairo.ImageSurface(cairo.Format.RGB24, *screen.get_size())
        tiledraw.context = cairo.Context(tiledraw.surface)
    context = tiledraw.context

    tilesize = 24*adjust

    context.set_operator(cairo.Operator.CLEAR)
    context.rectangle(0, 0, width, height)
    context.fill()
    context.set_operator(cairo.Operator.OVER)

    context.set_source_rgb(*hex2float(TILECOLOR))
    for coords, tile in sim.tile_dic.items():
        x, y = coords
        if tile == 1 or tile > 33:
            context.rectangle(x * tilesize, y * tilesize, tilesize, tilesize)
        elif tile > 1:
            if tile < 6:
                dx = tilesize/2 if tile == 3 else 0
                dy = tilesize/2 if tile == 4 else 0
                w = tilesize if tile % 2 == 0 else tilesize/2
                h = tilesize/2 if tile % 2 == 0 else tilesize
                context.rectangle(x * tilesize + dx, y * tilesize + dy, w, h)
            elif tile < 10:
                dx1 = 0
                dy1 = tilesize if tile == 8 else 0
                dx2 = 0 if tile == 9 else tilesize
                dy2 = tilesize if tile == 9 else 0
                dx3 = 0 if tile == 6 else tilesize
                dy3 = tilesize
                context.move_to(x * tilesize + dx1, y * tilesize + dy1)
                context.line_to(x * tilesize + dx2, y * tilesize + dy2)
                context.line_to(x * tilesize + dx3, y * tilesize + dy3)
            elif tile < 14:
                dx = tilesize if (tile == 11 or tile == 12) else 0
                dy = tilesize if (tile == 12 or tile == 13) else 0
                a1 = (math.pi / 2) * (tile - 10)
                a2 = (math.pi / 2) * (tile - 9)
                context.move_to(x * tilesize + dx, y * tilesize + dy)
                context.arc(x * tilesize + dx, y * tilesize + dy, tilesize, a1, a2)
                context.line_to(x * tilesize + dx, y * tilesize + dy)
            elif tile < 18:
                dx1 = tilesize if (tile == 15 or tile == 16) else 0
                dy1 = tilesize if (tile == 16 or tile == 17) else 0
                dx2 = tilesize if (tile == 14 or tile == 17) else 0
                dy2 = tilesize if (tile == 14 or tile == 15) else 0
                a1 = math.pi + (math.pi / 2) * (tile - 10)
                a2 = math.pi + (math.pi / 2) * (tile - 9)
                context.move_to(x * tilesize + dx1, y * tilesize + dy1)
                context.arc(x * tilesize + dx2, y * tilesize + dy2, tilesize, a1, a2)
                context.line_to(x * tilesize + dx1, y * tilesize + dy1)
            elif tile < 22:
                dx1 = 0
                dy1 = tilesize if (tile == 20 or tile == 21) else 0
                dx2 = tilesize
                dy2 = tilesize if (tile == 20 or tile == 21) else 0
                dx3 = tilesize if (tile == 19 or tile == 20) else 0
                dy3 = tilesize/2
                context.move_to(x * tilesize + dx1, y * tilesize + dy1)
                context.line_to(x * tilesize + dx2, y * tilesize + dy2)
                context.line_to(x * tilesize + dx3, y * tilesize + dy3)
            elif tile < 26:
                dx1 = 0
                dy1 = tilesize/2 if (tile == 23 or tile == 24) else 0
                dx2 = 0 if tile == 23 else tilesize
                dy2 = tilesize/2 if tile == 25 else 0
                dx3 = tilesize
                dy3 = (tilesize/2 if tile == 22 else 0) if tile < 24 else tilesize
                dx4 = tilesize if tile == 23 else 0
                dy4 = tilesize
                context.move_to(x * tilesize + dx1, y * tilesize + dy1)
                context.line_to(x * tilesize + dx2, y * tilesize + dy2)
                context.line_to(x * tilesize + dx3, y * tilesize + dy3)
                context.line_to(x * tilesize + dx4, y * tilesize + dy4)
            elif tile < 30:
                dx1 = tilesize/2
                dy1 = tilesize if (tile == 28 or tile == 29) else 0
                dx2 = tilesize if (tile == 27 or tile == 28) else 0
                dy2 = 0
                dx3 = tilesize if (tile == 27 or tile == 28) else 0
                dy3 = tilesize
                context.move_to(x * tilesize + dx1, y * tilesize + dy1)
                context.line_to(x * tilesize + dx2, y * tilesize + dy2)
                context.line_to(x * tilesize + dx3, y * tilesize + dy3)
            elif tile < 34:
                dx1 = tilesize/2
                dy1 = tilesize if (tile == 30 or tile == 31) else 0
                dx2 = tilesize if (tile == 31 or tile == 33) else 0
                dy2 = tilesize
                dx3 = tilesize if (tile == 31 or tile == 32) else 0
                dy3 = tilesize if (tile == 32 or tile == 33) else 0
                dx4 = tilesize if (tile == 30 or tile == 32) else 0
                dy4 = 0
                context.move_to(x * tilesize + dx1, y * tilesize + dy1)
                context.line_to(x * tilesize + dx2, y * tilesize + dy2)
                context.line_to(x * tilesize + dx3, y * tilesize + dy3)
                context.line_to(x * tilesize + dx4, y * tilesize + dy4)
        context.fill()

    buffer = tiledraw.surface.get_data()
    return pygame.image.frombuffer(buffer, screen.get_size(), "BGRA")

def entitydraw(init):
    if init:
        entitydraw.surface = cairo.ImageSurface(cairo.Format.RGB24, *screen.get_size())
        entitydraw.context = cairo.Context(entitydraw.surface)
    context = entitydraw.context

    context.set_source_rgb(*hex2float(BGCOLOR))
    context.rectangle(0, 0, width, height)
    context.fill()

    context.set_source_rgb(*hex2float(TILECOLOR))
    context.set_line_width(DOORWIDTH*adjust)
    for cell in sim.segment_dic.values():
        for segment in cell:
            if segment.active and segment.type == "linear" and not segment.oriented:
                context.move_to(segment.x1*adjust, segment.y1*adjust)
                context.line_to(segment.x2*adjust, segment.y2*adjust)
        context.stroke()

    context.set_source_rgb(*hex2float(ENTITYCOLOR))
    context.set_line_width(PLATFORMWIDTH*adjust)
    for entity in sim.entity_list:
        if entity.active:
            x = entity.xpos*adjust
            y = entity.ypos*adjust
            if hasattr(entity, "normal_x") and hasattr(entity, "normal_y"):
                if hasattr(entity, "RADIUS"):
                    radius = entity.RADIUS*adjust
                if hasattr(entity, "SEMI_SIDE"):
                    radius = entity.SEMI_SIDE*adjust
                angle = math.atan2(entity.normal_x, entity.normal_y) + math.pi / 2
                context.move_to(x + math.sin(angle) * radius, y + math.cos(angle) * radius)
                context.line_to(x - math.sin(angle) * radius, y - math.cos(angle) * radius)
                context.stroke()
            elif not hasattr(entity, "orientation") or entity.is_physical_collidable:
                if hasattr(entity, "RADIUS"):
                    radius = entity.RADIUS*adjust
                    context.arc(x, y, radius, 0, 2 * math.pi)
                    context.fill()
                elif hasattr(entity, "SEMI_SIDE"):
                    radius = entity.SEMI_SIDE*adjust
                    context.rectangle(x - radius, y - radius, radius * 2, radius * 2)
                    context.fill()

    context.set_source_rgb(*hex2float(NINJACOLOR))
    context.set_line_width(NINJAWIDTH*adjust)
    context.set_line_cap(cairo.LineCap.ROUND)
    bones = sim.ninja.bones
    segments = [[bones[limb[0]], bones[limb[1]]] for limb in LIMBS]
    radius = sim.ninja.RADIUS*adjust
    x = sim.ninja.xpos*adjust
    y = sim.ninja.ypos*adjust
    for segment in segments:
        x1 = segment[0][0]*2*radius + x
        y1 = segment[0][1]*2*radius + y
        x2 = segment[1][0]*2*radius + x
        y2 = segment[1][1]*2*radius + y
        context.move_to(x1, y1)
        context.line_to(x2, y2)
        context.stroke()

    buffer = entitydraw.surface.get_data()
    return pygame.image.frombuffer(buffer, screen.get_size(), "BGRA")

while running:
    resize = False
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.VIDEORESIZE or sim.frame == 0:
            resize = True

    keys = pygame.key.get_pressed()
    hor_input = 0
    jump_input = 0
    if keys[pygame.K_RIGHT]:
        hor_input = 1
    if keys[pygame.K_LEFT]:
        hor_input = -1
    if keys[pygame.K_z]:
        jump_input = 1
    if keys[pygame.K_SPACE]:
        sim.load(mapdata)

    adjust = min(screen.get_width()/SRCWIDTH, screen.get_height()/SRCHEIGHT)
    width, height = SRCWIDTH*adjust, SRCHEIGHT*adjust

    xoff = (screen.get_width() - width)/2
    yoff = (screen.get_height() - height)/2
    screen.fill("#"+TILECOLOR)
    screen.blit(entitydraw(resize), (xoff, yoff))
    screen.blit(tiledraw(resize), (xoff, yoff))
    pygame.draw.rect(screen, "#"+TILECOLOR, (xoff-2, yoff-2, width+4, height+4), 2)

    sim.tick(hor_input, jump_input)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()