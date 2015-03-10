import logging
logger = logging.getLogger(__name__)

import time
from functools import partial
from matplotlib.pyplot import figure, show, draw
from matplotlib.patches import Rectangle
from matplotlib.animation import FuncAnimation

def demo(drone):
    def keypress(ev):
        op = kmap_press.get(ev.key)
        if op is not None: op()
    def keyrelease(ev):
        op = kmap_release.get(ev.key)
        if op is not None: op()
    def disp(n):
        img.set_array(drone.image)
        img.changed()
        info(**drone.navdata[0])
    kmap_press, kmap_release = initkeys(drone)
    infoh = .15 #  infobar height (inches)
    imgw = 8. # image width (inches)
    s = drone.image.shape[:2]
    imgh = s[0]*imgw/s[1]
    q = imgh/(imgh+infoh)
    fig = figure(figsize=(imgw,imgh+infoh))
    fig.canvas.toolbar.setVisible(False)
    fig.canvas.callbacks.callbacks.clear()
    fig.canvas.mpl_connect('key_press_event',keypress)
    fig.canvas.mpl_connect('key_release_event',keyrelease)
    axinfo = fig.add_axes((0,q,1,1-q),xticks=(),yticks=())
    info = Info(axinfo)
    ax = fig.add_axes((0,0,1,q),xticks=(),yticks=(),frame_on=False)
    img = ax.imshow(drone.image)
    a = FuncAnimation(fig,func=disp,interval=100)
    show()
    drone.halt()

def Info(ax):
    draw()
    def width(t,w=ax.get_window_extent().width):
      ax.draw_artist(t)
      return t.get_window_extent().width/w
    style = dict(transform=ax.transAxes,va='center',ha='left',color='black',size='small')
    pos = sep = .01
    t = ax.text(pos,0.5,'time:',**style)
    pos += width(t)+sep
    clock_txt = ax.text(pos,0.5,'000.0',**style)
    pos += width(clock_txt)+2*sep
    t = ax.text(pos,0.5,'bat:',**style)
    pos += width(t)+sep
    bat_txt = ax.text(pos,0.5,'00',**style)
    pos += width(bat_txt)+sep
    bat_width = 0.1
    ax.add_patch(Rectangle((pos,.1),bat_width,.8,transform=ax.transAxes,fill=False,ec='black'))
    bat_rec = ax.add_patch(Rectangle((pos,.1),0.,.8,transform=ax.transAxes,fill=True,lw=0,fc='white'))
    pos += bat_width+2*sep
    t = ax.text(pos,0.5,'alt:',**style)
    pos += width(t)+sep
    alt_txt = ax.text(pos,0.5,'',**style)
    tref = time.time()
    def info(battery=None,altitude=None,**ka):
        clock_txt.set_text('{:.1f}'.format(time.time()-tref))
        bat_txt.set_text(str(battery))
        bat_rec.set_width(bat_width*battery/100.)
        bat_rec.set_fc('green' if battery>30 else 'orange' if battery>20 else 'red')
        alt_txt.set_text(str(altitude))
    return info

KeyMap = {
  'enter': 'takeoff',
  ' ': 'land',
  'escape': 'reset',
  '!up': 'move_forward',
  '!down': 'move_backward',
  '!left': 'move_left',
  '!right': 'move_right',
  '!ctrl+up': 'move_up',
  '!ctrl+down': 'move_down',
  '!ctrl+right': 'turn_right',
  '!ctrl+left': 'turn_left',
  '1': ('set_speed',.1),
  '2': ('set_speed',.2),
  '3': ('set_speed',.3),
  '4': ('set_speed',.4),
  '5': ('set_speed',.5),
  '6': ('set_speed',.6),
  '7': ('set_speed',.7),
  '8': ('set_speed',.8),
  '9': ('set_speed',.9),
  '0': ('set_speed',1.),
}
def initkeys(drone):
    kmap = {},{}
    for k,v in KeyMap.items():
        if k[0] == '!': k = k[1:]; kmap[1][k] = drone.hover
        if isinstance(v,str): v = getattr(drone,v)
        elif isinstance(v,tuple): v = partial(getattr(drone,v[0]),*v[1:])
        else: v = v(drone)
        kmap[0][k] = v
    return kmap

