#!/usr/bin/env python3
"""Render the README animation from the same synthetic scene as sensor_sim_node."""

from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
import math

WIDTH, HEIGHT = 960, 540
SCALE = 44
ORIGIN = (WIDTH // 2 + 80, HEIGHT // 2)


def screen(point):
    return (int(ORIGIN[0] + point[0] * SCALE), int(ORIGIN[1] - point[1] * SCALE))


def box(draw, bounds, color, width=3):
    x0, y0, x1, y1 = bounds
    pts = [screen((x0, y0)), screen((x1, y0)), screen((x1, y1)), screen((x0, y1))]
    draw.line(pts + [pts[0]], fill=color, width=width)


def frame(index):
    image = Image.new("RGB", (WIDTH, HEIGHT), "#07101f")
    draw = ImageDraw.Draw(image)
    for x in range(0, WIDTH, 32):
        draw.line((x, 0, x, HEIGHT), fill="#122340")
    for y in range(0, HEIGHT, 32):
        draw.line((0, y, WIDTH, y), fill="#122340")

    box(draw, (-9, -6, 9, 6), "#244d73", 3)
    for obstacle in [(-2.5, -1.2, -0.7, 1.8), (2, -4.2, 3.3, -1), (3.7, 1.5, 6.2, 3.6)]:
        box(draw, obstacle, "#63e5d2", 4)

    t = index * 0.32
    phase = 0.12 * t + 0.35
    pose = (5.1 * math.cos(phase), 3.3 * math.sin(phase))
    yaw = math.atan2(0.396 * math.cos(phase), -0.612 * math.sin(phase))
    robot = screen(pose)
    for beam in range(0, 360, 12):
        angle = yaw + math.radians(beam)
        radius = 2.1 + 1.25 * (0.5 + 0.5 * math.sin(beam * 0.17 + t))
        end = screen((pose[0] + radius * math.cos(angle), pose[1] + radius * math.sin(angle)))
        draw.line((robot, end), fill="#175a72", width=1)
        draw.ellipse((end[0]-2, end[1]-2, end[0]+2, end[1]+2), fill="#57dfff")

    targets = [
        (1.5 + 1.6 * math.sin(0.35 * t), 0.4 + 1.1 * math.cos(0.35 * t)),
        (-4 + 0.9 * math.cos(0.55 * t), -2.7 + 0.7 * math.sin(0.55 * t)),
    ]
    for target in targets:
        p = screen(target)
        draw.ellipse((p[0]-9, p[1]-9, p[0]+9, p[1]+9), fill="#ff5576", outline="#ffd1da", width=2)
        draw.line((robot, p), fill="#71304c", width=2)

    nose = screen((pose[0] + 0.42 * math.cos(yaw), pose[1] + 0.42 * math.sin(yaw)))
    left = screen((pose[0] + 0.28 * math.cos(yaw + 2.45), pose[1] + 0.28 * math.sin(yaw + 2.45)))
    right = screen((pose[0] + 0.28 * math.cos(yaw - 2.45), pose[1] + 0.28 * math.sin(yaw - 2.45)))
    draw.polygon((nose, left, right), fill="#37f1d1", outline="#eafffb")

    draw.rounded_rectangle((22, 20, 342, 142), radius=12, fill="#0d1930", outline="#263d64", width=2)
    draw.text((42, 37), "AstraMap / live simulation", fill="#f4f8ff", font=ImageFont.load_default())
    draw.text((42, 69), "CYAN  LiDAR + occupancy map", fill="#65f5da", font=ImageFont.load_default())
    draw.text((42, 93), "RED   radar dynamic tracks", fill="#ff718b", font=ImageFont.load_default())
    draw.text((42, 117), "20 Hz ROS 2 sensor graph", fill="#a9b9d6", font=ImageFont.load_default())
    return image


def mapping_frame(index):
    image = Image.new("RGB", (960, 420), "#07101f")
    draw = ImageDraw.Draw(image)
    for x in range(0, 960, 24):
        draw.line((x, 0, x, 420), fill="#10223c")
    for y in range(0, 420, 24):
        draw.line((0, y, 960, y), fill="#10223c")

    origin = (480, 220)
    walls = [(170, 80, 790, 80), (790, 80, 790, 350), (790, 350, 170, 350),
             (170, 350, 170, 80), (310, 135, 310, 290), (650, 165, 650, 315)]
    reveal = min(len(walls), 1 + index // 4)
    for wall in walls[:reveal]:
        draw.line(wall, fill="#4ee9d0", width=5)

    sweep = -math.pi + index * (2 * math.pi / 30)
    for n in range(48):
        angle = sweep - 0.9 + n * 1.8 / 47
        radius = 105 + 55 * (0.5 + 0.5 * math.sin(n * 0.8 + index * 0.23))
        end = (origin[0] + radius * math.cos(angle), origin[1] + radius * math.sin(angle))
        draw.line((origin, end), fill="#15566d", width=1)
        draw.ellipse((end[0]-2, end[1]-2, end[0]+2, end[1]+2), fill="#58c9ff")

    robot = [(origin[0]+18, origin[1]), (origin[0]-13, origin[1]-12), (origin[0]-13, origin[1]+12)]
    draw.polygon(robot, fill="#37f1d1", outline="#ecfffb")
    draw.rounded_rectangle((24, 22, 296, 108), radius=10, fill="#0d1930", outline="#263d64", width=2)
    draw.text((42, 40), "LOG-ODDS OCCUPANCY UPDATE", fill="#f4f8ff", font=ImageFont.load_default())
    draw.text((42, 67), "FREE CELLS  -0.42", fill="#58c9ff", font=ImageFont.load_default())
    draw.text((42, 88), "HIT CELLS   +0.85", fill="#37f1d1", font=ImageFont.load_default())
    return image


def main():
    assets = Path(__file__).resolve().parents[1] / "assets"
    output = assets / "demo.gif"
    frames = [frame(i) for i in range(30)]
    frames[0].save(output, save_all=True, append_images=frames[1:], duration=85, loop=0,
                   optimize=True)
    print(output)
    mapping_output = assets / "mapping.gif"
    mapping_frames = [mapping_frame(i) for i in range(30)]
    mapping_frames[0].save(mapping_output, save_all=True,
                           append_images=mapping_frames[1:], duration=90, loop=0,
                           optimize=True)
    print(mapping_output)


if __name__ == "__main__":
    main()
