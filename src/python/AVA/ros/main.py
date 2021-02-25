import pygame
from dynamixel_helper import DxlHelper
from Arm import Arm, ArmPositionController
from constants import *

# Define some colors.
BLACK = pygame.Color('black')
WHITE = pygame.Color('white')

arm = Arm(ids, offsets)
controller = ArmPositionController(arm, rotation_motor)
arm.set_torque(ids, True)

torque = True
position = (0, 0)
rotation = 180

arm.set_angle(12, 190)
controller.rotate(180)
controller.move(position[0], position[1], delay=1)

class TextPrint(object):
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 20)

    def tprint(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, (self.x, self.y))
        self.y += self.line_height

    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 15

    def indent(self):
        self.x += 10

    def unindent(self):
        self.x -= 10


pygame.init()

# Set the width and height of the screen (width, height).
screen = pygame.display.set_mode((500, 700))

pygame.display.set_caption("My Game")

# Loop until the user clicks the close button.
done = False

# Used to manage how fast the screen updates.
clock = pygame.time.Clock()

# Initialize the joysticks.
pygame.joystick.init()

# Get ready to print.
textPrint = TextPrint()


# -------- Main Program Loop -----------
while not done:
    for event in pygame.event.get(): # User did something.
        if event.type == pygame.QUIT: # If user clicked close.
            done = True # Flag that we are done so we exit this loop.
        elif event.type == pygame.JOYBUTTONDOWN:
            print("Joystick button pressed.")
        elif event.type == pygame.JOYBUTTONUP:
            print("Joystick button released.")

    screen.fill(WHITE)
    textPrint.reset()

    # Get count of joysticks.
    joystick_count = pygame.joystick.get_count()

    textPrint.tprint(screen, "Number of joysticks: {}".format(joystick_count))
    textPrint.indent()

    # For each joystick:
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()

        try:
            jid = joystick.get_instance_id()
        except AttributeError:
            # get_instance_id() is an SDL2 method
            jid = joystick.get_id()
        textPrint.tprint(screen, "Joystick {}".format(jid))
        textPrint.indent()

        # Get the name from the OS for the controller/joystick.
        name = joystick.get_name()
        textPrint.tprint(screen, "Joystick name: {}".format(name))

        try:
            guid = joystick.get_guid()
        except AttributeError:
            # get_guid() is an SDL2 method
            pass
        else:
            textPrint.tprint(screen, "GUID: {}".format(guid))

        # Usually axis run in pairs, up/down for one, and left/right for
        # the other.
        axes = joystick.get_numaxes()
        textPrint.tprint(screen, "Number of axes: {}".format(axes))
        textPrint.indent()
        move = False
        
        if abs(joystick.get_axis(1)) >= 0.1:
            move = True
            position = (position[0] + joystick.get_axis(1)*-0.5, position[1])
            
        if abs(joystick.get_axis(3)) >= 0.1:
            move = True
            position = (position[0], position[1] + joystick.get_axis(3)*-0.5)
        
        if abs(joystick.get_axis(2)) >= 0.1:
            rotation += joystick.get_axis(2) * -2
            controller.rotate(rotation)

        if move:
            controller.move(position[0], position[1])

        # arm.move(11, joystick.get_axis(2), -100)
        # arm.move(12, joystick.get_axis(3), 70)
        # arm.move(13, joystick.get_axis(1), 70)
        # arm.move(14, joystick.get_axis(0), 70)

        for i in range(axes):
            axis = joystick.get_axis(i)
            textPrint.tprint(screen, "Axis {} value: {:>6.3f}".format(i, axis))
        textPrint.unindent()

        buttons = joystick.get_numbuttons()
        textPrint.tprint(screen, "Number of buttons: {}".format(buttons))
        textPrint.indent()
        
        torque_button = joystick.get_button(2)
        if torque_button == 1:
            torque = not torque
            arm.set_torque(ids, torque)
                
        estop = joystick.get_button(3)
        if estop == 1:
            quit()

        reboot_button = joystick.get_button(0)
        if reboot_button == 1:
            controller.start_reboot_sequence()
            position = (0, 0)
            
        # 11, 12
        if joystick.get_button(10) == 1:
            arm.move(15, 1, 70)
        elif joystick.get_button(11) == 1:
            arm.move(15, 1, -70)
        
        for i in range(buttons):
            button = joystick.get_button(i)
            textPrint.tprint(screen,
                             "Button {:>2} value: {}".format(i, button))
            
            
        textPrint.unindent()

        hats = joystick.get_numhats()
        textPrint.tprint(screen, "Number of hats: {}".format(hats))
        textPrint.indent()

        # Hat position. All or nothing for direction, not a float like
        # get_axis(). Position is a tuple of int values (x, y).
        for i in range(hats):
            hat = joystick.get_hat(i)
            textPrint.tprint(screen, "Hat {} value: {}".format(i, str(hat)))
        textPrint.unindent()

        textPrint.tprint(screen, "Position: {}".format(position))
        textPrint.tprint(screen, "Rotation: {}".format(rotation))
        textPrint.tprint(screen, "Torque: {}".format(torque))

        textPrint.unindent()        
        #values = arm.get_motor_encoders()
        #for value in values:    
        #    textPrint.tprint(screen, f"ID {value[0]}: Position: {value[1]}")
        
        
        
        
        
    #
    # ALL CODE TO DRAW SHOULD GO ABOVE THIS COMMENT
    #

    # Go ahead and update the screen with what we've drawn.
    pygame.display.flip()

    # Limit to 20 frames per second.
    clock.tick(30)

# Close the window and quit.
# If you forget this line, the program will 'hang'
# on exit if running from IDLE.
pygame.quit()
