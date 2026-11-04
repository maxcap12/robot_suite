from numpy import disp
import os
import pygame
import sys
from cv_bridge import CvBridge
import cv2
from tello_control_station.video_container import VideoContainer
from ament_index_python import get_package_share_directory

matching_keys = {
    "j": pygame.K_j,
    "k": pygame.K_k,
    "t": pygame.K_t,
    "l": pygame.K_l,
    "q": pygame.K_q,
    "e": pygame.K_e,
    "h": pygame.K_h,
    "f": pygame.K_f,
    "m": pygame.K_m,
    "n": pygame.K_n,
    "w": pygame.K_w,
    "s": pygame.K_s,
    "a": pygame.K_a,
    "d": pygame.K_d,
    "up": pygame.K_UP,
    "down": pygame.K_DOWN,
    "left": pygame.K_LEFT,
    "right": pygame.K_RIGHT,
}


class Interface:
    def __init__(self, commands_image: str = "commands.png"):
        """commands_image: gesture table shown on the right, file name in the
        package share directory or absolute path."""
        pygame.init()
        pygame.joystick.init()

        self.joysticks = []
        self.is_clicked = False

        display_info = pygame.display.Info()
        # self.width = display_info.current_w // 2
        self.scale_factor = 0.90
        self.width = int(display_info.current_w * self.scale_factor)
        self.height = int(display_info.current_h * self.scale_factor) - 100

        self.display = pygame.display.set_mode((self.width, self.height))
        self.display.fill((35, 37, 40))
        pygame.display.flip()

        self.cv_bridge = CvBridge()
        self.video_container = VideoContainer(self.display)
        pkg_path = get_package_share_directory("tello_control_station")

        self.commands_img = pygame.image.load(os.path.join(pkg_path, commands_image))

        self.font = pygame.font.SysFont("Comic Sans MS", 30)

    def tick(self):
        self.video_container.update_image_frame()

        self.display.blit(
            self.commands_img, (self.width - 1 - self.commands_img.get_width(), 0)
        )

        mouse_clicked = False

        for event in pygame.event.get():

            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            elif event.type == pygame.JOYDEVICEADDED:
                print("joystick added")
                self.joysticks.append(pygame.joystick.Joystick(0))
                continue

            elif event.type == pygame.JOYDEVICEREMOVED:
                print("joystick removed")
                self.joysticks.pop(0)
                continue

            elif event.type == pygame.MOUSEBUTTONDOWN:
                mouse_clicked = True

        self.is_clicked = True if mouse_clicked and not self.is_clicked else False

    def get_key_pressed(self):
        return pygame.key.get_pressed()

    def get_joysticks(self):
        return self.joysticks

    def get_mouse_pos(self):
        return pygame.mouse.get_pos()

    def update_display_mode(self, mode):
        self.video_container.display_mode = mode

    def update_battery(self, battery_level):
        self.video_container.update_battery(battery_level)

    def update_bg_image(self, image):
        image = self.cv_bridge.imgmsg_to_cv2(image, "rgb8")

        img_h = len(image)
        img_w = len(image[0])

        scale = min(self.width / img_w, self.height / img_h)

        image = cv2.resize(
            image,
            (int(img_w * scale), int(img_h * scale)),
            interpolation=cv2.INTER_LINEAR,
        )

        opencv_image = image[:, :, ::-1]
        shape = image.shape[1::-1]
        self.video_container.update_bg_image(
            pygame.image.frombuffer(opencv_image.tostring(), shape, "BGR")
        )

    def update_boxes(self, info, is_selected_face, nose=None):
        if is_selected_face:
            self.video_container.update_selected_face_box(
                pygame.Rect(info.face.x, info.face.y, info.face.w, info.face.h),
                nose,
            )
        else:
            self.video_container.update_face_boxes(
                [pygame.Rect(el.face.x, el.face.y, el.face.w, el.face.h) for el in info]
            )

    def update_skeleton_layer(self, layer):
        image = self.cv_bridge.imgmsg_to_cv2(layer)
        img_h = len(image)
        img_w = len(image[0])

        scale = min(self.width / img_w, self.height / img_h)

        image = cv2.resize(
            image,
            (int(img_w * scale), int(img_h * scale)),
            interpolation=cv2.INTER_LINEAR,
        )
        self.video_container.update_skeleton_layer(
            pygame.image.frombuffer(image.tostring(), image.shape[1::-1], "RGB")
        )
