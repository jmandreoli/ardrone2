import numpy
import time
import threading

class DummyDrone (object):

    def __init__(self):
        self.image = numpy.zeros((360, 640, 3),'uint8')
        self.navdata = [dict(battery=1.,altitude=0)]
        self.network = DummyNetwork(self)

    def __getattr__(self,attr):
        def args(a,ka):
            for v in a: yield str(v)
            for k,v in ka.items(): yield '{}={}'.format(k,v)
        def f(*a,**ka): print('calling {}({})'.format(attr,','.join(args(a,ka))))
        return f

    def set_image(self,image):
        self.image = image
    
    def set_navdata(self,navdata):
        self.navdata = navdata

    def halt(self):
        self.network.halt()

class DummyNetwork (object):
    def __init__(self,drone):
        def run():
            import time
            u = numpy.ones(self.drone.image.shape,dtype=int)
            R = numpy.arange(0,256,10)
            b=1.
            while True:
                for t in R:
                    if not self.running: return
                    self.drone.set_image(t*u)
                    self.drone.set_navdata([dict(battery=int(100*b),altitude=t)])
                    b *= .97
                    time.sleep(.3)
        self.drone = drone
        self.running = True
        self.thread = threading.Thread(target=run)
        self.thread.daemon = True
        self.thread.start()
    def halt(self):
        self.running = False
        self.thread.join()
