#!/usr/bin/env python
#
""" BigRiver -- data flow synthesizer  

Test application to create a synthetic time series of 
an elevator cabine operation.

Creates physically based measurements of cabin velocity and 
motor temperature.

Units of measurements:
-- all times are in seconds Unix time 
        (seconds since 00:00:00 UTC on 1 January 1970)
-- distance in meters
-- velocities are in m/s
-- acceleration in m/s^2
-- temperature in Celsius 


relevant parameters and definitions (see CabinEvent class)

    a cabin event is a movement between two floors

    t0: initial time of  cabin event (cabin moves) 
    dt: event duration (travel time) 
    t1: final time (cabin stops) 
    v0: terminal velocity 
    tk: dynamical time scale -- typical raising time for velocity

    Tamb: ambient temperature 
    Tss:  motor steady state temperature 
    tau:  motor thermodynamic time scale

See documentation for details of the model.


"""
__author__    = "Fabricio Ferrari"
__copyright__ = "Copyright 2023, Projeto iTec RAE -- FURG "
__version__   = "0.3"
__email__     = "fabricio.ferrari@furg.br"



import numpy as np 
import time
import sys 
import os
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import paho.mqtt.client as mqtt #https://www.eclipse.org/paho/index.php?page=clients/python/docs/index.php


mqttBroker ="localhost" 


class CabinEvent:
    '''
    Main class that holds cabin events. 
    the parameters are
    
    '''
    # Class variable.
    autoEvent  = False
    startEvent = False

    def __init__(self, v0, t0, dt, tk, Tamb=22, Tss=80, tau=10):
        self.t0 = t0     # event start time
        self.dt = dt     # event duration 
        self.t1 = t0+dt  # event end time
        self.v0 = v0     # asymptotic velocity
        self.tk = tk     # dynamic time scale
        self.k  = 1/tk   # inverse time scale

        self.Tamb = Tamb  # ambient temperature
        self.Tss  = Tss   # steady state on temperature
        self.tau  = tau   # thermodynamic time constant

        Tlast = 0.0

        

    def velocity(self, t):
        self.v = self.v0 * (self.logistic_eq(t,self.t0) - self.logistic_eq(t,self.t1))
        return self.v

    def Temperature(self, t):
        # if time t is an array
        if type(t) is np.ndarray: 
            T1 = np.where(  t < self.t0,                    self.Tamb,                               t)
            T2 = np.where( (T1 >= self.t0) & (T1 < self.t1), self.heating_eq(t, self.Tss, self.Tamb), T1)
            # last temperature relative from which to cool down
            Tlast =  (T2[(T1 >= self.t0) & (T1 < self.t1)][-1])
            T3 = np.where(  T2 >= self.t1,                  self.cooling_eq(t, Tlast, self.Tamb), T2 )
            return T3
        # else if it is a single value
        else:
            if   ( t < self.t0 ):                # before event
                T = self.Tamb
            elif (self.t0 <= t and t < self.t1): # heating
                T = self.heating_eq(t, self.Tss, self.Tamb)
            elif ( t >= self.t1 ) : 
                T = self.cooling_eq(t, self.Tss, self.Tamb)
            return(T)

        

    def logistic_eq(self, t,tc):
        return (1 / (1+ np.exp(-self.k*(t-tc)) ) )


    def heating_eq(self, t, Tsup, Tinf):
        # calculates heating equation at given time t  with Tsup superior 
        # Tinf inferior pleateau temperatures with given tau time constant
        T = (Tsup-Tinf) * (1 - np.exp(-(t-self.t0)/self.tau) ) + Tinf  
        self.Tlast = T
        return T

    def cooling_eq(self, t, Tsup, Tinf):
        # same as heating_eq  for cooling equation
        Tsup = self.Tlast
        T = (Tsup-Tinf) * np.exp(-(t-self.t1)/self.tau)  + Tinf  
        return T


def on_connect(client, userdata, flags, rc):
    # This will be called once the client connects
    print(f"Connected with result code {rc}")
    # Subscribe here!
    #client.subscribe("esp32/updatetime")
    client.subscribe("cabin/startEvent")
    client.subscribe("cabin/autoEvent")

def on_message(client, userdata, msg):
    print(f"Message received [{msg.topic}]: {msg.payload}")

    if msg.topic == 'cabin/autoEvent' and msg.payload == b'false':
        CabinEvent.autoEvent = False
    else:     
        CabinEvent.autoEvent = True

    if msg.topic == 'cabin/startEvent' and msg.payload == b'true':
        CabinEvent.startEvent = True



if __name__ == '__main__':

    client = mqtt.Client("BigRiver")
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(mqttBroker) 
    client.loop_start()  # Start networking daemon
    print('MQTT: Connected and active')


    begintime0 = time.time()
    tau = 10  # 
    v0  = 1.2 # typical cabin velocity: 1.2 m/s
    x   = 0.0  # cabin actual position     
    samplingtime = 1.0 

    while (1):
        
        while(CabinEvent.autoEvent == True):
            print('autoEvent', CabinEvent.autoEvent)
        

            begintime = time.time()
            t0        = begintime + random.randrange(5,10)
            dt        = random.randrange(4,20) # travel time or how many levels
            v0sign    = random.choice([-1,1])  # upward or downward velocity
            Ntau      = random.randrange(1,5) # how many tau times to wait until next event

            evento    = CabinEvent(v0*v0sign, t0, dt, tk=1.0, tau=tau )

            tnow = time.time()
            while(tnow < t0+dt+Ntau*tau and  CabinEvent.autoEvent == True):
                tnow = time.time()
                V = evento.velocity(tnow)
                T = evento.Temperature(tnow)
                x = x + V * samplingtime

                if 'debug' in sys.argv:
                    print (tnow, V, T  )
                else:
                    print(".")

                client.publish("motor/temperature", T)
                client.publish("cabin/velocity", V)
                client.publish("cabin/position", x)
                
                time.sleep(samplingtime)

        time.sleep(1)


