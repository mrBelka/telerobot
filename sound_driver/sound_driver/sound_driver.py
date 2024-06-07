import pyglet

def main(args=None):
    sound = pyglet.media.load("sound.wav")
    sound.play()

if __name__ == '__main__':
    main()