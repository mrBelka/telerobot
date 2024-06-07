import pygame

def play_wav_file(file_path):
    pygame.init()
    pygame.mixer.init()
    sound = pygame.mixer.Sound(file_path)
    sound.play()
    pygame.time.wait(int(sound.get_length()) * 1000)

def main(args=None):
    file_path = "/home/robot/ros2_ws/src/telerobot/sound_driver/sound_driver/sound.wav"
    play_wav_file(file_path)
    
if __name__ == '__main__':
    main()
