import time
import urllib.parse
import urllib.request

import pygame


SPOT_API_BASE = "http://127.0.0.1:8002"

FORWARD_SPEED = 1.5
TURN_SPEED = 1.2

COMMAND_HZ = 20
WINDOW_SIZE = (420, 180)

STOP_COMMAND = (0.0, 0.0, 0.0)



# --- HTTP Commands ---
def post(endpoint: str, params: dict | None = None) -> None:
    params = params or {}
    query = urllib.parse.urlencode(params)
    url = f"{SPOT_API_BASE}{endpoint}"

    if query:
        url = f"{url}?{query}"

    request = urllib.request.Request(url, method="POST")

    with urllib.request.urlopen(request, timeout=0.2) as response:
        response.read()

def send_cmd_vel(vx: float, vy: float, wz: float) -> None:
    post(
        "/cmd_vel",
        {
            "vx": vx,
            "vy": vy,
            "wz": wz,
        },
    )

def send_stop() -> None:
    post("/stop")


# --- Keyboard Mapping ---
def get_command_from_keys() -> tuple[float, float, float]:
    keys = pygame.key.get_pressed()

    vx = 0.0
    vy = 0.0
    wz = 0.0

    if keys[pygame.K_w]:
        vx += FORWARD_SPEED
    if keys[pygame.K_s]:
        vx -= FORWARD_SPEED

    if keys[pygame.K_a]:
        wz -= TURN_SPEED
    if keys[pygame.K_d]:
        wz += TURN_SPEED

    return vx, vy, wz


# --- UI ---
def draw_window(screen: pygame.Surface, font: pygame.font.Font) -> None:
    screen.fill((20, 20, 20))

    lines = [
        "Spot WASD Teleop",
        "W/S: forward/back",
        "A/D: turn left/right",
        "Q/E: strafe left/right",
        "Space: stop",
        "Esc: quit",
    ]

    y = 15
    for line in lines:
        text = font.render(line, True, (230, 230, 230))
        screen.blit(text, (20, y))
        y += 24

    pygame.display.flip()


# ---------- Main Loop ----------
def main() -> None:
    pygame.init()

    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("Spot WASD Teleop")
    font = pygame.font.SysFont(None, 24)
    clock = pygame.time.Clock()

    running = True
    last_cmd = None
    last_send_time = 0.0

    print("WASD teleop started.")
    print("Click/focus the pygame window, then use W/A/S/D.")
    print("Press Esc to quit.")

    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        send_stop()
                        last_cmd = STOP_COMMAND

            cmd = get_command_from_keys()
            now = time.monotonic()

            should_send = (
                cmd != last_cmd
                or now - last_send_time >= 1.0 / COMMAND_HZ
            )

            if should_send:
                if cmd == STOP_COMMAND:
                    if last_cmd != STOP_COMMAND:
                        send_stop()
                else:
                    send_cmd_vel(*cmd)

                last_cmd = cmd
                last_send_time = now

            draw_window(screen, font)
            clock.tick(COMMAND_HZ)

    finally:
        send_stop()
        pygame.quit()
        print("Stopped Spot and closed teleop.")



if __name__ == "__main__":
    main()