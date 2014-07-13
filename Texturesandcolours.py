import Image
from visual import *

'''
##################
Colours and Images
##################
'''
#imports two images we need to use 
im2 = Image.open('floor.png') 
im = Image.open('libkoki.png')



im3 =  Image.open('back.png')
im4 = Image.open('leftside.png')
im5 = Image.open('top.png')
im6 =Image.open('front.png')
im7 = Image.open('rightside.png')

#converts images to usable textures
tex = materials.texture(data=im, mapping='sign')
tex2 = materials.texture(data=im2, mapping='sign')
tex3 = materials.texture(data=im3, mapping='rectangular')
tex4 = materials.texture(data=im4, mapping='rectangular')
tex5 = materials.texture(data=im5, mapping='rectangular')
tex6 = materials.texture(data=im6, mapping='rectangular')
tex7 = materials.texture(data=im7, mapping='rectangular')

#defines any useful colours we need, RGB scale between 0-1, user colour_mapper function to convert from 0-255 to 0-1
color.brown = (0.38,0.26,0.078)
color.orange = (0.85,0.54,0.18)