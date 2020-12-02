import pygame, sys

Clock = pygame.time.Clock()
from pygame.locals import *
pygame.init()
pygame.display.set_caption('Hexapod : Team chenil')
screen = pygame.display.set_mode((1000, 750))

pygame.mixer.music.load('baha.ogg')
pygame.mixer.music.play(-1)

font = pygame.font.SysFont(None, 20)
 
def draw_text(text, font, color, surface, x, y):
    textobj = font.render(text, 1, color)
    textrect = textobj.get_rect()
    textrect.topleft = (x, y)
    surface.blit(textobj, textrect)
 
click = False
 
def main_menu():
    

    while True:
 
        screen.fill((0,0,0))
        #Chargement et collage du fond
        # fond = pygame.image.load("Bordeaux.png")
        # rect = fond.get_rect()
        # screen.blit(fond,rect)
        # pygame.display.flip()
        # continuer = 1
        # while continuer:
	    #     continuer = int(input())

        draw_text('MENU', font, (255, 255, 255), screen, 20, 20)
 
        mx, my = pygame.mouse.get_pos()
 
        button_1 = pygame.Rect(820, 100, 50, 50)
        draw_text('Moving an arbitrary leg to an arbitrary (x, y, z) position', font, (255, 255, 255), screen, 30, 125)
        button_2 = pygame.Rect(820, 200, 50, 50)
        draw_text('Moving the center of the robot to an arbitrary (x, y, z) position (the 6 legs staying on the floor)', font, (255, 255, 255), screen, 30, 225)
        button_3 = pygame.Rect(820, 300, 50, 50)
        draw_text('Walking in a straight line', font, (255, 255, 255), screen, 30, 325)
        button_4 = pygame.Rect(820, 400, 50, 50)
        draw_text('Rotating without moving the center of the robot', font, (255, 255, 255), screen, 30, 425)
        button_5 = pygame.Rect(820, 500, 50, 50)
        draw_text('Moving as a Crazy Frog', font, (255, 255, 255), screen, 30, 525)
        if button_1.collidepoint((mx, my)):
            if click:
                mode1()
        if button_2.collidepoint((mx, my)):
            if click:
                mode2()
        if button_3.collidepoint((mx, my)):
            if click:
                mode3()
        if button_4.collidepoint((mx, my)):
            if click:
                mode4()
        if button_5.collidepoint((mx, my)):
            if click:
                mode5()

        pygame.draw.rect(screen, (255, 0, 0), button_1)
        pygame.draw.rect(screen, (255, 0, 0), button_2)
        pygame.draw.rect(screen, (255, 0, 0), button_3)
        pygame.draw.rect(screen, (255, 0, 0), button_4)
        pygame.draw.rect(screen, (255, 0, 0), button_5)
 
        click = False
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    pygame.quit()
                    sys.exit()
            if event.type == MOUSEBUTTONDOWN:
                if event.button == 1:
                    click = True
 
        pygame.display.update()
        Clock.tick(60)
 
def mode1():
    running = True
    while running:
        screen.fill((0,0,0))
        
        draw_text('Moving an arbitrary leg to an arbitrary (x, y, z) position', font, (255, 255, 255), screen, 20, 20)
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False
        
        pygame.display.update()
        Clock.tick(60)
 
def mode2():
    running = True
    while running:
        screen.fill((0,0,0))
        draw_text('Moving the center of the robot to an arbitrary (x, y, z) position (the 6 legs staying on the floor)', font, (255, 255, 255), screen, 20, 20)
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False
        
        pygame.display.update()
        Clock.tick(60)

def mode3():
    running = True
    while running:
        screen.fill((0,0,0))
 
        draw_text('Walking in a straight line', font, (255, 255, 255), screen, 20, 20)
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False
        
        pygame.display.update()
        Clock.tick(60)

def mode4():
    running = True
    while running:
        screen.fill((0,0,0))
 
        draw_text('Rotating without moving the center of the robot', font, (255, 255, 255), screen, 20, 20)
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False
        
        pygame.display.update()
        Clock.tick(60)

def mode5():
    running = True
    while running:
        screen.fill((0,0,0))
 
        draw_text('crazy Frog', font, (255, 255, 255), screen, 20, 20)
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False
        
        pygame.display.update()
        Clock.tick(60)


main_menu()