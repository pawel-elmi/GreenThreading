import webapp
from threading import Timer, Thread, Event
from eventlet import tpool

import eventlet

cnt = 0

def main():

    dupa = eventlet.greenthread.spawn(webapp.webappRun)
    while(True):
        eventlet.sleep(0.1)
        elo = eventlet.greenthread.spawn_after(2, delayedPrint)
        
        
def check_function():
    print("Eloszka")
    
    
def delayedPrint():
    global cnt
    cnt = cnt + 1
    webapp.socketio.emit('someEvent',str(cnt))
    print(str(cnt + 1))



if __name__ == '__main__':
    main()



#     print("dupka")
#     eventlet.sleep(1)
#     print("blada")
#     eventlet.sleep(1)

#     stopFlagChecker = Event()
#     CheckingThread(stopFlagChecker).start()

#     webapp_thread = Thread(target = webapp.webappRun)     
#     webapp_thread.start()
    
#     tpool.execute(webapp.webappRun)



#     
# class CheckingThread(Thread):
#     def __init__(self,event):
#         Thread.__init__(self)
#         self.stopped = event
#     def run(self):
#         while not self.stopped.wait(1):
#             check_function()
            


    
#     stopFlagA = Event()
#     CheckingThread(stopFlagChecker).start()

#     check_thread = Thread(target = check_function)
#     check_thread.start()