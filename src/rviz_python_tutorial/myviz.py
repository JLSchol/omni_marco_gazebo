#!/usr/bin/env python


## First we start with the standard ros Python import line:
import roslib; roslib.load_manifest('rviz_python_tutorial')

## Then load sys to get sys.argv.
import sys

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

## Finally import the RViz bindings themselves.
import rviz
# python 2.7
from itertools import izip as zip


## The MyViz class is the main container widget.
class MyViz( QWidget ):

    ## MyViz Constructor
    ## ^^^^^^^^^^^^^^^^^
    ##
    ## Its constructor creates and configures all the component widgets:
    ## frame, thickness_slider, top_button, and front_button, and adds them
    ## to layouts.
    def __init__(self):
        QWidget.__init__(self)

        """
        'setAccessibleDescription', 'setAccessibleName', 'setAnimated',  
        'setAutoFillBackground', 'setBackgroundRole', 'setBaseSize', 'setCentralWidget', 'setContentsMargins', 
        'setContextMenuPolicy', 'setDisabled', 'setDockNestingEnabled', 'setDockOptions', 
        'setEnabled', 
        'setHidden', 'setHideButtonVisibility', 'setIconSize', 'setInputMethodHints', , 'setLocale', 'setMask', 'setMaximumHeight', 'setMaximumSize', 'setMaximumWidth', 
        'setMenuBar', 'setMenuWidget', 'setMinimumHeight', 'setMinimumSize', 'setMinimumWidth', 'setMouseTracking', 
        'setObjectName', 'setPalette', 'setParent', 'setProperty', 'setShortcutAutoRepeat', 'setShortcutEnabled', 
        'setShowChooseNewMaster', 'setSizeIncrement', 'setSizePolicy', 'setSplashPath', 'setStatusBar', 'setStatusTip', 
        'setStyle', 'setStyleSheet', 'setTabOrder', 'setTabPosition', 'setTabShape', 'setToolButtonStyle', 
        'setToolTip', 'setToolTipDuration', 'setUnifiedTitleAndToolBarOnMac', 'setUpdatesEnabled', 'setVisible', 
        'setWhatsThis', 'setWindowFilePath', 'setWindowFlags', 'setWindowIcon', 'setWindowIconText', 'setWindowModality', 
        'setWindowModified', 'setWindowOpacity', 'setWindowRole', 'setWindowState', 'setWindowTitle'

        """

        ######################################################################################################
        ##################################  RVIZ SHIT #########################################################
        ######################################################################################################
        # get frames from config
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile( config, "simple_experiment.rviz" )
        self.topFrame = self.createFrame(config)
        self.botFrame = self.createFrame(config)


        # get visualization manager for each frame
        self.topManager = self.topFrame.getManager()
        self.botManager = self.botFrame.getManager()

        # get view manager
        self.topViewManager = self.topManager.getViewManager()
        self.botViewManager = self.botManager.getViewManager()

        # get dict with keyNames and numberValues
        self.views = self.getViewDict(self.topViewManager) # same for both

        # set start views
        self.setView(self.topViewManager, self.views, "TopView")
        self.setView(self.botViewManager, self.views, "FrontView15")


        ######################################################################################################
        ##################################  GUI SHIT #########################################################
        ######################################################################################################

        # initialize some params
        buttonNames = ["front","front 15","front 30","front 45","top 30","top 15","top","front 3D","top 3D","left"]
        buttonCBs = [self.front, self.front15, self.front30, self.front45, 
                    self.top30, self.top15, self.top, self.front3D, self.top3D, self.left]


        # Overal structure of layout
        self.layout = QVBoxLayout()  

        # TOP SUBLAYOUT
        topSubLayout = QVBoxLayout()
        # add rviz frame to top sub layout
        topSubLayout.addWidget(self.topFrame)
        # add button row to sub layout
        topButtonLayout, self.topRefs = self.createButtonRowLayout(buttonNames, buttonCBs)
        topSubLayout.addLayout(topButtonLayout)

        # BOT SUBLAYOUT
        botSubLayout = QVBoxLayout()
        # add rviz frame to bot sub layout
        botSubLayout.addWidget(self.botFrame)
        # add button row to sub layout
        # need to add view manageger to 
        botButtonLayout, self.botRefs = self.createButtonRowLayout(buttonNames, buttonCBs)
        botSubLayout.addLayout(botButtonLayout)
        # find sender object
        # print(10*"------------")
        print(botButtonLayout.children()) #0x7f848856fb00>
        # print(10*"------------")
        # SLIDER WIDGET
        viewDistSlider = self.createHorSlider(50, 250, self.onSliderChange)


        # add everything to layout and set for visualization
        self.layout.addLayout(topSubLayout)
        self.layout.addLayout(botSubLayout)
        self.layout.addWidget(viewDistSlider)
        self.setLayout( self.layout )



################################ RVIZ HELPER FUNCTIONS ################################
    def createFrame(self, configObj):
        # initialize
        frame = rviz.VisualizationFrame()
        frame.setSplashPath( "" )
        frame.initialize()
        frame.load(configObj)
        ## You can also store any other application data you like in
        ## the config object.  Here we read the window title from the
        ## map key called "Title", which has been added by hand to the
        ## config file.
        # self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )
        # settings
        frame.setMenuBar( None )
        frame.setStatusBar( None )
        frame.setHideButtonVisibility( False )
        return frame

    def getViewDict(self,viewManager):
        # self.views = {
        #     "FrontView":0   ,"FrontView15":1,
        #     "FrontView30":2 ,"FrontView45":3,
        #     "TopView30":4   ,"TopView15":5,
        #     "TopView":6     ,"LeftView":7,
        #     "3DViewFront":8 ,"3DViewTop":9
        # }
        valueIndex = range(viewManager.getNumViews())
        keyNames = [viewManager.getViewAt(index).getName() for index in valueIndex]
        viewDict = {name: index for name, index in zip(keyNames,valueIndex)}
        return viewDict

    def setView(self,viewManager,viewDict,name):
        number = viewDict[name]
        viewManager.setCurrentFrom(viewManager.getViewAt(number))

    def switchToView( self, index, manager ):
        viewManager = manager.getViewManager()
        viewManager.setCurrentFrom( viewManager.getViewAt(index))

################################ LAYOUT FUNCTIONS ################################
    def createButtonRowLayout(self,buttonNames,onClickList):
        butLayout = QHBoxLayout()
        refList = []
        for i,name in enumerate(buttonNames):
            but = QPushButton(name)
            but.clicked.connect(onClickList[i])
            print(10*"------------")
            butLayout.addWidget(but)
            refList.append(but)
        return butLayout, refList

    def createHorSlider(self,minVal,maxVal,eventCB):
        slider = QSlider(Qt.Horizontal)
        slider.setTracking(True)
        slider.setMinimum(minVal)
        slider.setMaximum(maxVal)
        slider.valueChanged.connect(eventCB)
        return slider

################################ BUTTON CALLBACKS AND OTHER GUI EVENTS ################################
    def onSliderChange( self, new_value ):
        # topViewManager.setCurrentFrom( topViewManager.getViewAt( 0 ))
        if self.topViewManager != None and self.botViewManager != None:
            for viewMan in [self.topViewManager, self.botViewManager]:
                print(viewMan.getCurrent().subProp("Distance").getValue())
                viewMan.getCurrent().subProp( "Distance" ).setValue( new_value/100.0)
                viewMan.setCurrentFrom( viewMan.getCurrent())

    def getManager(self,iets):
        botRow = True
        if botrow:
            return self.botManager
        else:
            return self.topManager


    def front( self ):
        view = "FrontView"
        manager = self.topManager
        if self.layout.sender() in self.botRefs:
            manager = self.botManager

        # print(self.layout.sender())
        # print(self.layout.parent())
        # print(self.layout.findChildren())
        # print(self.layout.children())
        # print(10*"------")
        # manager = self.getManager(iets)

        self.switchToView( self.views[view], manager );
    def front15( self ):
        view = "FrontView15"
        self.switchToView( self.views[view], self.topManager );
    def front30( self ):
        view = "FrontView30"
        self.switchToView( self.views[view], self.topManager );
    def front45( self ):
        view = "FrontView45"
        self.switchToView( self.views[view], self.topManager );
    def front3D( self ):
        view = "3DViewFront"
        self.switchToView( self.views[view], self.topManager );

    def top( self ):
        view = "TopView"
        self.switchToView( self.views[view], self.topManager );
    def top15( self ):
        view = "TopView15"
        self.switchToView( self.views[view], self.topManager );
    def top30( self ):
        view = "TopView30"
        self.switchToView( self.views[view], self.topManager );
    def top3D( self ):
        view = "3DViewTop"
        self.switchToView( self.views[view], self.topManager );

    def left( self ):
        view = "LeftView"
        self.switchToView( self.views[view], self.topManager ); 





## Start the Application
## ^^^^^^^^^^^^^^^^^^^^^
##
## That is just about it.  All that's left is the standard Qt
## top-level application code: create a QApplication, instantiate our
## class, and start Qt's main event loop (app.exec_()).
if __name__ == '__main__':
    app = QApplication( sys.argv )

    myviz = MyViz()
    myviz.resize( 500, 500 )
    myviz.show()

    app.exec_()
