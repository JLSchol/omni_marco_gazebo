#!/usr/bin/env python

## BEGIN_TUTORIAL
##
## Imports
## ^^^^^^^
##
## First we start with the standard ros Python import line:
import roslib; roslib.load_manifest('rviz_python_tutorial')

## Then load sys to get sys.argv.
import sys

## Next import all the Qt bindings into the current namespace, for
## convenience.  This uses the "python_qt_binding" package which hides
## differences between PyQt and PySide, and works if at least one of
## the two is installed.  The RViz Python bindings use
## python_qt_binding internally, so you should use it here as well.
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

## Finally import the RViz bindings themselves.
import rviz


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

        ## rviz.VisualizationFrame is the main container widget of the
        ## regular RViz application, with menus, a toolbar, a status
        ## bar, and many docked subpanels.  In this example, we
        ## disable everything so that the only thing visible is the 3D
        ## render window.
        self.frame = rviz.VisualizationFrame()
        self.frame2 = rviz.VisualizationFrame()

        ## The "splash path" is the full path of an image file which
        ## gets shown during loading.  Setting it to the empty string
        ## suppresses that behavior.
        self.frame.setSplashPath( "" )
        self.frame2.setSplashPath( "" )
    
        ## VisualizationFrame.initialize() must be called before
        ## VisualizationFrame.load().  In fact it must be called
        ## before most interactions with RViz classes because it
        ## instantiates and initializes the VisualizationManager,
        ## which is the central class of RViz.
        self.frame.initialize()
        self.frame2.initialize()

        ## The reader reads config file data into the config object.
        ## VisualizationFrame reads its data from the config object.
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        reader.readFile( config, "simple_experiment.rviz" )
        self.frame.load( config )
        self.frame2.load( config )
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
        # self.frame2.setHidden(True)
        
        ## You can also store any other application data you like in
        ## the config object.  Here we read the window title from the
        ## map key called "Title", which has been added by hand to the
        ## config file.
        self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )

        # self.frame.setFullScreen( True )

        ## Here we disable the menu bar (from the top), status bar
        ## (from the bottom), and the "hide-docks" buttons, which are
        ## the tall skinny buttons on the left and right sides of the
        ## main render window.
        # self.frame.setToolTip( None )
        self.frame.setMenuBar( None )
        self.frame2.setMenuBar( None )
        self.frame.setStatusBar( None )
        self.frame2.setStatusBar( None )
        self.frame.setHideButtonVisibility( False )
        self.frame2.setHideButtonVisibility( False )

        ## frame.getManager() returns the VisualizationManager
        ## instance, which is a very central class.  It has pointers
        ## to other manager objects and is generally required to make
        ## any changes in an rviz instance.
        self.manager1 = self.frame.getManager()
        # self.manager1.getToolManager().removeAll()
        self.viewManager1 = self.manager1.getViewManager()
        self.viewManager1.setCurrentFrom( self.viewManager1.getViewAt( 0 ))

        self.manager2 = self.frame2.getManager()
        # self.manager2.getToolManager().removeAll()
        self.viewManager2 = self.manager2.getViewManager()
        self.viewManager2.setCurrentFrom( self.viewManager2.getViewAt( 1 ))

        ## Since the config file is part of the source code for this
        ## example, we know that the first display in the list is the
        ## grid we want to control.  Here we just save a reference to
        ## it for later.
        self.grid_display = self.manager1.getRootDisplayGroup().getDisplayAt( 0 )
        # self.topview = self.manager1.getViewManager().getViewAt( 0 )

        
        
        ## Here we create the layout and other widgets in the usual Qt way.
        layout = QVBoxLayout()


        h_views = QVBoxLayout()
        h_views.addWidget(self.frame)
        h_views.addWidget(self.frame2)
        layout.addLayout(h_views)

        # view2 = QVBoxLayout()
        # view2.addWidget( self.frame2 )
        # layout.addWidget(view2)
        
        thickness_slider = QSlider( Qt.Horizontal )
        thickness_slider.setTracking( True )
        thickness_slider.setMinimum( 1 )
        thickness_slider.setMaximum( 100 )
        thickness_slider.valueChanged.connect( self.onThicknessSliderChanged )
        layout.addWidget( thickness_slider )
        h_layout = QHBoxLayout()
        
        top_button = QPushButton( "Top View" )
        top_button.clicked.connect( self.onTopButtonClick )
        h_layout.addWidget( top_button )
        
        front_button = QPushButton( "Front view" )
        front_button.clicked.connect( self.onSideButtonClick )
        h_layout.addWidget( front_button )
        
        layout.addLayout( h_layout )
        
        self.setLayout( layout )

    ## Handle GUI events
    ## ^^^^^^^^^^^^^^^^^
    ##
    ## After the constructor, for this example the class just needs to
    ## respond to GUI events.  Here is the slider callback.
    ## rviz.Display is a subclass of rviz.Property.  Each Property can
    ## have sub-properties, forming a tree.  To change a Property of a
    ## Display, use the subProp() function to walk down the tree to
    ## find the child you need.
    def onThicknessSliderChanged( self, new_value ):
        # viewManager1.setCurrentFrom( viewManager1.getViewAt( 0 ))
        if self.viewManager1 != None and self.viewManager2 != None:

            for viewMan in [self.viewManager1, self.viewManager2]:
                # print(new_value)
                print(viewMan.getViewAt( 0 ).subProp("Distance").getValue())
                viewMan.getViewAt( 0 ).subProp( "Scale" ).setValue( 10*(101 - new_value))
                viewMan.setCurrentFrom( viewMan.getViewAt( 0 ))


    ## The view buttons just call switchToView() with the name of a saved view.
    def onTopButtonClick( self ):
        self.switchToView( "TopView", self.manager1 );
        
    def onSideButtonClick( self ):
        self.switchToView( "FrontView", self.manager2 );
        
    ## switchToView() works by looping over the views saved in the
    ## ViewManager and looking for one with a matching name.
    ##
    ## view_man.setCurrentFrom() takes the saved view
    ## instance and copies it to set the current view
    ## controller.
    def switchToView( self, view_name, manager ):
        view_man = manager.getViewManager()
        for i in range( view_man.getNumViews() ):
            if view_man.getViewAt( i ).getName() == view_name:
                view_man.setCurrentFrom( view_man.getViewAt( i ))
                return
        print( "Did not find view named %s." % view_name )

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
