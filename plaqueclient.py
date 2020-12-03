#!/bin/python3


import random
import time
import threading
import Pyro4.core
import RPi.GPIO as GPIO
import os
import board
import neopixel
import serial
import operator

time.sleep(1)
colors=[(0, 0, 0),(84, 0, 0),(0, 0, 84),(0, 84, 0),(84, 84, 0),(0, 84, 84),(84, 0, 84),(84, 84, 84)]
#           black0   red1       blue2     green3       yellow4      cyan5       megenta6       white7
# red=(64, 0, 0)
# blue=(0, 0, 64)
# green=(0, 64, 0)
# black=(0, 0, 0)
# yellow=(64, 64, 0)
# cyan=(0, 64, 64)
# rose=(64, 0, 64)
# white=(64, 64, 64)

class Client():
#classe Cable

    def __init__(self):
        random.seed()
        self.connect() #fonction de connexion au daemon pyro
        self.flag=[False,False,False] #initialisation des variables aux valeurs par défaut
        self.temps=[0,0,0]
        self.etat=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.etatleds=['0','0','0','0','0','0','0','0','0','0']
        self.etatbouton=[0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.pixels = neopixel.NeoPixel(board.D18, 53, auto_write=False)
        self.startpos=[(0,0),(1,0),(2,0),(3,-1),(3,-2),(3,-3),(2,-3),(1,-3),(0,-2),(0,-1)]
        self.startposled=[0,49,46,26,False,False,41,29,False,False,38,34,37,13]
        self.boutonpos=[(0,0),(1,0),(2,0),(0,-1),(1,-1),(2,-1),(3,-1),(0,-2),(1,-2),(2,-2),(3,-2),(1,-3),(2,-3),(3,-3)]
        self.chemin=[[0],[0],[0],[0],[0],[0],[0]]
        self.rgb=[False,False,False]
        self.flagvalidation=False

        self.boutonled=[\
        [False,[1,2],[24,25],False,False,[0,0,0]],\
        [False,[47,48],[3,4],[1,2],False,[0,0,0]],\
        [False,False,[44,45],[47,48],False,[0,0,0]],\
        [False,[22,23],[27,28],False,[24,25],[0,0,0]],\
        [False,[5,6],[21,20],[22,23],[4,3],[0,0,0]],\
        [False,[42,43],[7,8],[5,6],[44,45],[0,0,0]],\
        [False,False,[39,40],[42,43],False,[0,0,0]],\
        [False,[30,31],False,False,[27,28],[0,0,0]],\
        [False,[18,19],[32,33],[30,31],[20,21],[0,0,0]],\
        [False,[9,10],[16,17],[18,19],[7,8],[0,0,0]],\
        [False,False,[11,12],[9,10],[39,40],[0,0,0]],\
        [False,[35,36],False,False,[32,33],[0,0,0]],\
        [False,[14,15],False,[35,36],[16,17],[0,0,0]],\
        [False,False,False,[14,15],[11,12],[0,0,0]]\
        ]
        self.winpos=[[84, 0, 84],[84, 0, 0],[84, 84, 0],[84, 0, 84],[84, 0, 0],[84, 84, 0],\
        [84, 84, 0],[0, 84, 84],[0, 84, 84],[0, 84, 84],[84, 0, 0],[0, 0, 0],[0, 84, 84],[0, 0, 84]]
        try:
            self.ser = serial.Serial('/dev/ttyACM0', 19200)

        except:
            self.ser = serial.Serial('/dev/ttyACM1', 19200)
        self.ser.setTimeout=1.0
        self.ser.write_timeout=1.0
        time.sleep(2)



    def connect(self):
        connected=False
        while connected==False:
            try:
                self.ipserveur="192.168.0.190" #ip du serveur pyro


                self.nameserver=Pyro4.locateNS(host=self.ipserveur,port=9090)  #localisation du serveur de nom pyro4
                self.uri = self.nameserver.lookup("lostserver")   #recherche de la classe partagée ayant l'id  sur le serveur de nom
                self.server = Pyro4.Proxy(self.uri)  #liaison de la classe locale server à la classe partagée trouvée précédement
                connected=True #connexion réussie

            except:# sleep 5 secondes avant de retenter une connexion si serveur non accessible
                print("waiting for server")

                time.sleep(5)

    def refresh(self):  #refresh flags, state and temps from server each 500ms
        try:
            self.flag=self.server.getflag() #1: run #2 win #3 lost
            self.etat=self.server.getetat() #list of 16 int
            self.temps=self.server.gettemps() #1 time, #2 additionnal time #ending time

        except:  #reconnexion en cas de perte de la connexion avec le serveur pyro
            print("Connection lost. REBINDING...")
            print("(restart the server now)")
            self.server._pyroReconnect()
            time.sleep(5)
        threading.Timer(0.5, self.refresh).start()



    def serialin(self):
# reception etat des boutons via serial dans variable self.etatbouton [14] int 1-4
        #if script.etat[1]==0:
            etatboutonbuff=self.ser.readline().decode('utf-8', "ignore")

            if len(etatboutonbuff)==16:
                if self.flag[0]:
                    if self.etat[1]==0:
                        self.etatbouton[0]=int(etatboutonbuff[0])
                        self.etatbouton[1]=int(etatboutonbuff[2])
                        self.etatbouton[2]=int(etatboutonbuff[5])
                        self.etatbouton[3]=int(etatboutonbuff[1])
                        self.etatbouton[4]=int(etatboutonbuff[4])
                        self.etatbouton[5]=int(etatboutonbuff[7])
                        self.etatbouton[6]=int(etatboutonbuff[10])
                        self.etatbouton[7]=int(etatboutonbuff[3])
                        self.etatbouton[8]=int(etatboutonbuff[6])
                        self.etatbouton[9]=int(etatboutonbuff[9])
                        self.etatbouton[10]=int(etatboutonbuff[12])
                        self.etatbouton[11]=int(etatboutonbuff[8])
                        self.etatbouton[12]=int(etatboutonbuff[11])
                        self.etatbouton[13]=int(etatboutonbuff[13])

    #        print (self.etatbouton)
            threading.Timer(0.08, self.serialin).start()

    def serialout(self):
# envoie etat des led, script.etatled  [10] 0-1
        try:
            self.ser.write(b'z')

            self.ser.write(''.join(self.etatleds).encode("utf-8",'ignore'))

            self.ser.write(b'a')
        except:
            print("error")

    def cheminfinder(self,start,num):
#fonction récursive pour cheminement, start= numero du bouton, num = couleur 0 rouge 1 vert 2 bleu 3
            fini=False
            xybouton=self.boutonpos[start]
            if script.etatbouton[start]==1:

                next=self.boutonpos.index(tuple(map(sum, zip(xybouton, (1,0)))))
                self.boutonled[start][5][num]=84
                for i in self.chemin[num]:
                    if i ==next:
                        fini=True
                if fini==False:
                    self.chemin[num].append(next)

                    self.cheminfinder(next,num)


            elif script.etatbouton[start]==2:
                next=self.boutonpos.index(tuple(map(sum, zip(xybouton, (0,-1)))))
                self.boutonled[start][5][num]=84
                for i in self.chemin[num]:
                    if i ==next:
                        fini=True
                if fini==False:
                    self.chemin[num].append(next)

                    self.cheminfinder(next,num)

            elif script.etatbouton[start]==3:
                next=self.boutonpos.index(tuple(map(sum, zip(xybouton, (-1,0)))))
                self.boutonled[start][5][num]=84
                for i in self.chemin[num]:
                    if i ==next:
                        fini=True
                if fini==False:
                    self.chemin[num].append(next)

                    self.cheminfinder(next,num)
            elif script.etatbouton[start]==4:
                next=self.boutonpos.index(tuple(map(sum, zip(xybouton, (0,1)))))
                self.boutonled[start][5][num]=84
                for i in self.chemin[num]:
                    if i ==next:
                        fini=True
                if fini==False:
                    self.chemin[num].append(next)

                    self.cheminfinder(next,num)
            elif script.etatbouton[start]==5:

                self.boutonled[start][5][num]=84
                if start==0:
                    if tuple(self.boutonled[start][5])==self.pixels[50]:
                        self.rgb[0]=True
                    else:
                        self.rgb[0]=False
                if start==2:
                    if tuple(self.boutonled[start][5])==self.pixels[52]:
                        self.rgb[1]=True
                    else:
                        self.rgb[1]=False
                if start==7:
                    if tuple(self.boutonled[start][5])==self.pixels[51]:
                        self.rgb[2]=True
                    else:
                        self.rgb[2]=False

            elif script.etatbouton[start]==0:
                self.boutonled[start][5][num]=0
                if start==0:
                    self.rgb[0]=False
                if start==2:
                    self.rgb[1]=False
                if start==7:
                    self.rgb[2]=False
            #print (self.chemin[num])
script=Client()  #création de l'objet script de classe client
script.refresh()

script.serialin()
script.serialout()
script.ser.flush()

while True:
    try:
#boucle infinie
        time.sleep(0.15)
        #print (script.etatbouton)
        if script.flag[0]:
            if script.etat[1]==0:


                #remise a zéro couleurs strip et tableau
                script.pixels.fill((0, 0, 0))
                for i in range(14):
                    script.boutonled[i][5]=[0,0,0]

                #couleur des leds de départs entrée
                script.pixels[script.startposled[1]]=colors[1]
                script.pixels[script.startposled[3]]=colors[2]
                script.pixels[script.startposled[6]]=colors[3]
                script.pixels[script.startposled[10]]=colors[1]
                script.pixels[script.startposled[11]]=colors[1]
                script.pixels[script.startposled[12]]=colors[3]
                script.pixels[script.startposled[13]]=colors[2]



                #couleur des leds d'arrivée
                script.pixels[50]=tuple(map(sum, zip(list(script.pixels[script.startposled[1]]),list(script.pixels[script.startposled[3]]))))
                script.pixels[52]=tuple(map(sum, zip(list(script.pixels[script.startposled[10]]),list(script.pixels[script.startposled[6]]))))
                script.pixels[51]=tuple(map(sum, zip(list(script.pixels[script.startposled[12]]),list(script.pixels[script.startposled[13]]))))





                #7 points de départ des chemins entrée
                script.chemin[0]=[1]
                script.chemin[1]=[3]
                script.chemin[2]=[6]
                script.chemin[3]=[10]
                script.chemin[4]=[11]
                script.chemin[5]=[12]
                script.chemin[6]=[13]

                # lancement fonctionnement cheminement: (départ, couleur)
                script.cheminfinder(1,0)
                script.cheminfinder(3,2)
                script.cheminfinder(6,1)
                script.cheminfinder(10,0)
                script.cheminfinder(11,0)
                script.cheminfinder(12,1)
                script.cheminfinder(13,2)




                #validation si gagnée normalement
                if all(i==True for i in script.rgb) :
                    script.etatleds=['1','1','1','1','1','1','1','1','1','1']
                    script.serialout()
                    script.etat[1]=1
                    script.server.changeetat(1,1)
                    #script.etatbouton=[5, 3, 5, 0, 0, 4, 3, 5, 3, 3, 4, 0, 4, 3]
                    #print ("gagnée")
                    script.flagvalidation=True

                    #for i in range(14):
                        #print(script.boutonled[i][5])

                    #print (script.pixels[50])
                    #print (script.pixels[51])
                    #print (script.pixels[52])


            elif script.etat[1]==1 and script.flagvalidation==False:
                    #validation à distance depuis gm

                    script.etatleds=['1','1','1','1','1','1','1','1','1','1']
                    script.serialout()
                    script.etatbouton=[5, 2, 5, 4, 3, 4, 3, 5, 3, 3, 4, 0, 4, 3]
                    script.pixels.fill((0,0,0))
                    script.pixels[50]=(84, 0, 84)
                    script.pixels[52]=(0, 84, 84)
                    script.pixels[51]=(84, 84, 0)
                    for i in range(14):

                        script.boutonled[i][5]=script.winpos[i]

                    #print ("validé")

        if script.flag[0]==False:
            #reset
            #print ("not running")
            script.etatleds=['0','0','0','0','0','0','0','0','0','0']
            script.serialout()
            script.rgb=[False,False,False]
            script.etatbouton=[0,0,0,0,0,0,0,0,0,0,0,0,0,0]
            for i in range(14):
                script.boutonled[i][5]=[0,0,0]
            script.pixels.fill((0,0,0))
            script.flagvalidation=False
                #chemin des 14 boutons pour allumer la lumière en fonction des couleurs
        for i in range(14):
                    if script.etatbouton[i]==1:
                        script.pixels[script.boutonled[i][1][0]]=tuple(script.boutonled[i][5])
                        script.pixels[script.boutonled[i][1][1]]=tuple(script.boutonled[i][5])
                    elif script.etatbouton[i]==2:
                        script.pixels[script.boutonled[i][2][0]]=tuple(script.boutonled[i][5])
                        script.pixels[script.boutonled[i][2][1]]=tuple(script.boutonled[i][5])
                    elif script.etatbouton[i]==3:
                        script.pixels[script.boutonled[i][3][0]]=tuple(script.boutonled[i][5])
                        script.pixels[script.boutonled[i][3][1]]=tuple(script.boutonled[i][5])
                    elif script.etatbouton[i]==4:
                        script.pixels[script.boutonled[i][4][0]]=tuple(script.boutonled[i][5])
                        script.pixels[script.boutonled[i][4][1]]=tuple(script.boutonled[i][5])
                    elif script.etatbouton[i]==5:
                        script.pixels[script.startposled[i]]=tuple(script.boutonled[i][5])

        script.pixels.show()


#illumination bande led suivant boutons:

    except KeyboardInterrupt:
        script.ser.flush()
        script.ser.close()
