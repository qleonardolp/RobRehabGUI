#-*-coding:cp1252-*-

import os
import glob
import sys
import json
import struct

from bluetooth import *

from definitions import *

DEFAULT_UUID = '94f39d29-7d6d-437d-973b-fba39e49d4ee'

MESSAGE_TIMEOUT = 0.02

class Connection:

  clientSocket = None
  robotInfoString = bytearray( BUFFER_SIZE )
  setpointBuffer = bytearray( BUFFER_SIZE )

  def __init__( self ):
    self.serverSocket = BluetoothSocket( RFCOMM )
    self.serverSocket.bind( ( "", PORT_ANY ) )
    self.serverSocket.listen( 1 )
    
    port = self.serverSocket.getsockname()[ 1 ]

    print( "Waiting for connection on RFCOMM channel %d" % port )

  def __del__( self ):
    self.Disconnect()
    self.serverSocket.close()

  def Connect( self, host ):
    advertise_service( self.serverSocket, host,
                       service_id = DEFAULT_UUID,
                       service_classes = [ DEFAULT_UUID, SERIAL_PORT_CLASS ],
                       profiles = [ SERIAL_PORT_PROFILE ], 
                       protocols = [ OBEX_UUID ] 
                     )
    
    try:
      self.Disconnect()
      self.clientSocket, clientInfo = self.serverSocket.accept()
      self.clientSocket.settimeout( MESSAGE_TIMEOUT )
      print( 'accepted client from %s' % clientInfo )
    except:
      print( sys.exc_info() )
      self.clientSocket = None

  def Disconnect( self ):
    if self.clientSocket is not None:
      self.clientSocket.close()
      self.clientSocket = None

  def RefreshInfo( self ):
    robotsInfo = {}
    if self.clientSocket is not None:
      messageBuffer = bytearray( [ 0 ] )
      try:
        self.clientSocket.sendall( messageBuffer )
        robotInfoString = self.clientSocket.recv( BUFFER_SIZE )
        print( 'RefreshInfo: received JSON string: ' + str(robotInfoString, 'utf-8').strip( '\0' ) )
        robotsInfo = json.loads( str(robotInfoString, 'utf-8').strip( '\0' ) )
      except:
        print( sys.exc_info() )
        robotsInfo = {}

    robotID = robotsInfo.get( 'id', '' )
    jointsList = robotsInfo.get( 'joints', [] )
    axesList = robotsInfo.get( 'axes', [] )

    return ( robotID, ( jointsList, axesList ) )

  def SendCommand( self, commandKey ):
    if self.clientSocket is not None:
      messageBuffer = bytearray( [ commandKey ] )
      print( 'SendCommand: sending message buffer: ' + str(list(messageBuffer)) )
      try:
        self.clientSocket.sendall( messageBuffer )
      except:
        print( sys.exc_info() )

  def SetUser( self, userName ):
    if self.clientSocket is not None:
      messageBuffer = bytearray( [ SET_USER ] ) + userName.encode()
      print( 'SetUser: sending message buffer: ' + str(list(messageBuffer)) )
      try:
        self.clientSocket.sendall( messageBuffer )
      except:
        print( sys.exc_info() )

  def CheckState( self, eventNumber ):
    return False
    #return self.eventSocket.recv( BUFFER_SIZE )

  def _SendSetpoints( self, dataSocket, coordinateIndex, setpoints ):
    if self.clientSocket is not None:
      struct.pack_into( 'BB', self.setpointBuffer, 0, 1, coordinateIndex )
      for setpointIndex in range( len(setpoints) ):
        setpointOffset = 2 + setpointIndex * FLOAT_SIZE
        struct.pack_into( 'f', self.setpointBuffer, setpointOffset, setpoints[ setpointIndex ] )
      #print( '_SendSetpoints: sending message buffer: ' + str( list( self.setpointBuffer ) ) )
      try:
        dataSocket.sendall( self.setpointBuffer )
      except:
        #print( sys.exc_info() )
        pass

  def SendAxisSetpoints( self, axisIndex, setpoints ):
    self._SendSetpoints( self.axisSocket, axisIndex, setpoints )

  def _ReceiveMeasures( self, dataSocket, coordinateIndex, measures ):
    if self.clientSocket is not None:
      try:
        messageBuffer = dataSocket.recv( BUFFER_SIZE )
        coordinatesNumber = int( messageBuffer[ 0 ] )
        #print( '_ReceiveMeasures: received message buffer: ' + str( list( messageBuffer ) ) )
        for coordinate in range( coordinatesNumber ):
          dataOffset = coordinate * len(measures) * FLOAT_SIZE + 1
          if int( messageBuffer[ dataOffset ] ) == coordinateIndex:
            for measureIndex in range( len(measures) ):
              measureOffset = dataOffset + measureIndex * FLOAT_SIZE + 1
              measures[ measureIndex ] = struct.unpack_from( 'f', messageBuffer, measureOffset )[ 0 ]
            return True
      except:
        #print( sys.exc_info() )
        pass
    return False

  def ReceiveAxisMeasures( self, axisIndex, measures ):
    return self._ReceiveMeasures( self.clientSocket, axisIndex, measures )

  def ReceiveJointMeasures( self, jointIndex, measures ):
    return self._ReceiveMeasures( self.clientSocket, jointIndex, measures ) 
