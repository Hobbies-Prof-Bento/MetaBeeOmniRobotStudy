#! /usr/bin/env python3

import math

class InertialCalculator(object):

    def __init__(self):
        print("Calculadora de inércia iniciada")
    
    def start_ask_loop(self):

        selection = "start"

        while (selection !="S") or (selection !="s"):

            print("#######################")
            print("Seleciona a forma geométrica que pretende calcular:")
            print("[1] caixa (largura(w) * profundidade(d) * altura(h))")
            print("[2] esfera (raio(r))")
            print("[3] cilindro (raio(r) * altura(h))")
            print("[S] sair do programa")
            selection = input('escolha uma opção: ')
            self.select_action(selection)
        
        print("Encerrando a aplicação")

    def select_action(self, selection):
        if selection == "1":
            mass = float(input("massa>>"))
            width = float(input("largura>>"))
            depth = float(input("profundidade>>"))
            height = float(input("altura>>"))
            self.calculate_box_inertia(m=mass, w=width,d=depth, h=height)
        
        elif selection == "2":
            mass = float(input("massa>>"))
            radius = float(input("raio>>"))
            self.calculate_sphere_inertia(m=mass, r=radius)
        
        elif selection == "3":
            mass = float(input("massa>>"))
            radius = float(input("raio>>"))
            height = float(input("altura>>"))
            self.calculate_cylinder_inertia(m=mass, r=radius, h=height)
        
        elif selection == "S" or selection == "s":
            print("sair selecionado")
        
        else:
            print("Escolha inválida")

    def calculate_box_inertia(self, m, w, d, h):
        Iw = (m/12.0)*(pow(d,2)+pow(h,2))
        Id = (m/12.0)*(pow(w,2)+pow(h,2))
        Ih = (m/12.0)*(pow(w,2)+pow(d,2))

        print("Iw = "+str(Iw))
        print("Id = "+str(Id))
        print("Ih = "+str(Ih))

    def calculate_sphere_inertia(self, m, r):
        I = (2*m*pow(r,2))/5.0

        print("I = "+str(I))
    
    def calculate_cylinder_inertia(self, m, r, h):
        Ix = (m/12.0)*(3*pow(r,2)+pow(h, 2))
        Iy = Ix
        Iz = (m*pow(r, 2))/2.0

        print("Ix = "+str(Ix))
        print("Iy = "+str(Iy))
        print("Iz = "+str(Iz))

if __name__ == '__main__':
    inertial_object = InertialCalculator()
    inertial_object.start_ask_loop()