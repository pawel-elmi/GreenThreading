import CS5523
import webapp
import eventlet
# from flask import jsonify
# import json
import math


dupa = eventlet.greenthread.spawn(webapp.webappRun)
cnt = 0

JSONtoSend = {"CH1":0,"CH2":2}


def main():
    CS5523.Init()
    CS5523.pwm_fans.set_PWM_dutycycle(CS5523._PWM_FANS_PIN, 250)
    
    while(True):
            elo = eventlet.greenthread.spawn_after(1, CS5523.ReadTemperature )
            eventlet.sleep(0.3)
            helo = eventlet.greenthread.spawn( CS5523.Printer )
            hello = eventlet.greenthread.spawn( printToClient )
            
def printToClient():
    temp = { "CH1": CS5523.TEMPERATURE[0],
             "CH2": CS5523.TEMPERATURE[1],
             "CH3": CS5523.TEMPERATURE[2],
             "CH4": CS5523.TEMPERATURE[3],
             "PidOut": CS5523.pid_output,
             "P": round(CS5523.pid.PTerm,2),
             "I": round(CS5523.pid.ITerm,2),
             "D": round(CS5523.pid.DTerm,2),
             "Temp_diff": CS5523.TEMP_DIFF}
    webapp.socketio.emit('someEvent', temp)
    print(str(temp))
    
if __name__ == '__main__':
    main()
    

        
    
    
    
    
# --------------------- SMIETNIK ------------------------------ #
# 
# class SendingThread(Thread):
#     def __init__(self,event):
#         Thread.__init__(self)
#         self.stopped = event
#     def run(self):
#         while not self.stopped.wait(1):
#             if(len(webapp.clients) != 0):
#                 print(str(CS5523.TEMPERATURE[0]))
#                 print("Clients qt: ",len(webapp.clients))
#                 mesycz = str(CS5523.TEMPERATURE[0])
#                 webapp.send_message(mesycz) #webapp.clients[len(webapp.clients) - 1],
#                 webapp.sendMessage(mesycz)

# from flask_socketio import SocketIO

#import GUI
# from time import sleep

# import eventlet
# eventlet.monkey_patch()


#     stopFlagGUI = Event()
#     GUI.GUIThread(stopFlagGUI).start()

#    gui.mainloop()
#     CS5523.plt.show()

#     ani = animation.FuncAnimation(CS5523.fig, CS5523.PlotGraph, 1000)




#     try:
#         while(True):
#             if CS5523.ErrGetter() == 1:
#                 print("Cannot connect, exiting...")
#                 stopFlag.set()
#                 stopFlagPrinter.set()
#                 exit()
# #            a = input("Dupa: ")
# #            if (a == 'q'):
# #                CS5523.pwm_fridge.set_PWM_dutycycle(CS5523._PWM_PIN,0)
# #                CS5523.pwm_fans.set_PWM_dutycycle(CS5523._PWM_FANS_PIN, 0)
# #    CS5523.pwm_fridge.set_PWM_dutycycle(_PWM_PIN,0)
#     except(KeyboardInterrupt, SystemExit):
#         CS5523.pwm_fridge.set_PWM_dutycycle(CS5523._PWM_PIN,0)
#         CS5523.pwm_fans.set_PWM_dutycycle(CS5523._PWM_FANS_PIN, 0)

#        nb = input('Write PWM: ')
#        print ('PWM: %s \n' % (nb))
#        if CS5523.ErrGetter() == 2:
##            stopFlag.set()
##            stopFlagPrinter.set()
#            CS5523.ErrResetter(2)
#            CS5523.Init()
#            print("ADC frozen, reseting...")
#            print("Frozen cnt: ",CS5523.ErrResetter(3))
#            sleep(1)
#            sleep(1)
#            CS5523.ReaderThread(stopFlag).start()
#            CS5523.PrinterThread(stopFlagPrinter).start()