'''Tools to help with vision processsing.'''

import cscore as cs
import cv2
import numpy as np
from networktables import NetworkTables
from collections import namedtuple
from multiprocessing import Process, Queue

HSV = namedtuple('HSV', ('H', 'S', 'V'))
Threshhold = namedtuple('Threshhold', ('min', 'max'))
Color = namedtuple('Color', ('blue', 'green', 'red'))
Processor = namedtuple(
    'Processor',
    ('name', 'function', 'color', 'queue', 'process')
)
Target = namedtuple('Target', ('distance', 'x', 'y', 'contour'))
Drawable = namedtuple('Drawable', ('color', 'contours'))

NetworkTables.setServerTeam(2539)

colors = {
    'red': Color(0, 0, 255),
    'green': Color(0, 255, 0),
    'blue': Color(255, 0, 0),
    'purple': Color(255, 0, 255),
    'yellow': Color(0, 255, 255),
    'black': Color(0, 0, 0),
    'white': Color(255, 255, 255),
    'gray': Color(127, 127, 127),
    'gold': Color(50, 215, 255),
    'turquoise': Color(208, 224, 64),
    'pink': Color(255, 0, 255),
    'orange': Color(0, 100, 255),
    'cyan': Color(255, 255, 0)
}

class Camera:
    '''Wrapper class for handing cameras.'''

    swapColor1 = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
    swapColor2 = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
    swapBW1 = np.zeros(shape=(480, 640, 1), dtype=np.uint8)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    initialized = False
    cameras = []
    usedPorts = []
    nextPort = 8080


    def __init__(self, source):
        self.width = 640
        self.height = 480

        self.processors = {}
        self.newTargets = {}
        self.markers = {}
        self.markerAge = {}
        self.lastContours = {}
        self.contourAge = {}

        camera = cs.UsbCamera('usbcam', source)

        camera.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, 640, 480, 30)

        cvSink = cs.CvSink('cvsink')
        cvSink.setSource(camera)

        self.port = Camera.nextPort
        cvSource = cs.CvSource('cvsource', cs.VideoMode.PixelFormat.kMJPEG, 640, 480, 30)
        server = cs.MjpegServer('cvhttpserver', self.port)
        Camera.usedPorts.append(self.port)

        Camera.nextPort += 1

        server.setSource(cvSource)

        self.cvSink = cvSink
        self.cvSource = cvSource

        self.registerCamera(self)


    def addProcessor(self, name, hsv, processor, **kwargs):
        if kwargs.get('color'):
            color = kwargs['color']
        else:
            color = colors['white']

        processor = {
            'name': name,
            'function': processor,
            'color': color,
            'queue': Queue(),
            'process': Process()
        }

        try:
            self.processors[hsv].append(processor)
        except KeyError:
            self.processors[hsv] = [processor]
            self.contourAge[hsv] = 10
            self.markerAge[processor['name']] = 0


    def processImage(self):
        time, img = self.cvSink.grabFrame(Camera.swapColor1)
        if time == 0:
            print('Camera error: ', self.cvSink.getError())
        else:
            img = Camera.undistort(img)
            self.cvSource.putFrame(self.drawTargets(img))

            for hsv, processors in self.processors.items():
                self.contourAge[hsv] += 1
                for processor in processors:
                    while not processor['queue'].empty():
                        self.newTargets[processor['name']].append(processor['queue'].get())

                    if not processor['process'].is_alive():
                        self.registerTargets(processor)
                        self.newTargets[processor['name']] = []
                        processor['process'] = Process(
                            target=processor['function'],
                            args=(self.contours(img, hsv), processor['queue'])
                        )
                        processor['process'].start()


    def debugImage(self):
        time, img = self.cvSink.grabFrame(Camera.swapColor1)
        if time == 0:
            print('Camera error: ', self.cvSink.getError())
        else:
            Camera.img = Camera.undistort(img)

            for hsv, processors in self.processors.items():
                contours = self.contours(Camera.img, hsv)
                cv2.drawContours(Camera.img, contours, -1, colors['white'])

                for processor in processors:
                    processor.process = Process(
                        target=processor.function,
                        args=(self.contours(Camera.img, hsv), processor.queue)
                    )
                    processor.process.start()

                    self.newTargets[processor.name] = []
                    while not processor.queue.empty():
                        self.newTargets[processor.name].append(processor.queue.get())

                    processor.process.join()
                    self.registerTargets(processor)

            self.cvSource.putFrame(self.drawTargets(Camera.img))


    def contours(self, img, hsv):
        if self.contourAge[hsv] <= 3:
            return self.lastContours[hsv]

        cv2.cvtColor(img, cv2.COLOR_BGR2HSV, dst=Camera.swapColor2)
        cv2.inRange(Camera.swapColor2, hsv.min, hsv.max, dst=Camera.swapBW1)
        cv2.erode(Camera.swapBW1, Camera.kernel, dst=Camera.swapBW1)
        cv2.dilate(Camera.swapBW1, Camera.kernel, dst=Camera.swapBW1)

        Camera.swapBW1, contours, hierarchy = cv2.findContours(
            Camera.swapBW1,
            cv2.RETR_LIST,
            cv2.CHAIN_APPROX_SIMPLE
        )

        self.contourAge[hsv] = 0
        self.lastContours[hsv] = contours
        return self.lastContours[hsv]


    @classmethod
    def registerCamera(cls, camera):
        if cls.initialized == False:
            fs = cv2.FileStorage('back_camera_data.xml', cv2.FILE_STORAGE_READ)
            cameraMatrix = fs.getNode('Camera_Matrix').mat()
            distortionCoefficients = fs.getNode('Distortion_Coefficients').mat()

            def undistort(img):
                cv2.undistort(
                    img,
                    cameraMatrix,
                    distortionCoefficients,
                    dst=cls.swapColor2
                )

                return cls.swapColor2

            cls.undistort = undistort

            cls.initialized = True
            cls.nt = NetworkTables.getTable('vision')
            cls.debug = cls.nt.getAutoUpdateValue('debug', False)

        cls.nt.putNumberArray('ports', cls.usedPorts)
        cls.cameras.append(camera)


    def registerTargets(self, processor):
        try:
            targets = self.newTargets[processor['name']]
        except KeyError:
            Camera.nt.putBoolean('%s/found' % processor['name'], False)
            return

        if len(targets) == 0:
            self.markerAge[processor['name']] += 1
            if self.markerAge[processor['name']] > 5:
                try:
                    del self.markers[processor['name']]
                except KeyError:
                    pass
                Camera.nt.putBoolean('%s/found' % processor['name'], False)

            return

        self.markerAge[processor['name']] = 0
        contours = []
        distances = []
        x = []
        y = []
        for target in targets:
            distances.append(target.distance)
            x.append(target.x)
            y.append(target.y)
            contours.append(target.contour)

        Camera.nt.putBoolean('%s/found' % processor['name'], True)
        Camera.nt.putNumberArray('%s/distance' % processor['name'], distances)
        Camera.nt.putNumberArray('%s/x' % processor['name'], x)
        Camera.nt.putNumberArray('%s/y' % processor['name'], y)
        self.markers[processor.name] = Drawable(processor['color'], contours)


    def drawTargets(self, img):
        for drawables in self.markers.values():
            for color, contours in drawables:
                cv2.drawContours(img, contours, -1, color, 2)

        return img


    @classmethod
    def startVision(cls):
        if cls.debug.value:
            while cls.debug.value:
                for camera in cls.cameras:
                    camera.debugImage()

        else:
            while not cls.debug.value:
                for camera in cls.cameras:
                    camera.processImage()
