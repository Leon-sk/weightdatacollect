#!/usr/local/bin/env python3
# -*-  coding:utf-8 -*-
'''
Created on Sep 9, 2020

@author: zhwm
@note: update weight config
'''

import configparser
import json
import os
import sys
import uuid

WEIGHTCONF = '/etc/weight/config.ini'
AUTHCONF = '/data/config.json'


def loadDeviceAuth( auth_conf ):
    auth = {}
    try:
        with open( auth_conf, 'r' ) as f:
            auth = json.load( f )
    except Exception as ex:
        print( ex )
    return auth


def updateWeightConf( weight_ini, deviceId ):
    try:
        config = configparser.ConfigParser()
        config.read( weight_ini, encoding = 'utf8' )

        RGBFilePath = '/data/weight/{0}/rgb/'.format( deviceId )
        os.makedirs( RGBFilePath, exist_ok = True )
        config['DEFAULT']['RGBFilePath'] = RGBFilePath

        DepthFilePath = '/data/weight/{0}/depth/'.format( deviceId )
        os.makedirs( DepthFilePath, exist_ok = True )
        config['DEFAULT']['DepthFilePath'] = DepthFilePath

        DepthShowFilePath = '/data/weight/{0}/depthshow/'.format( deviceId )
        os.makedirs( DepthShowFilePath, exist_ok = True )
        config['DEFAULT']['DepthShowFilePath'] = DepthShowFilePath

        PointCloudFilePath = '/data/weight/{0}/point/'.format( deviceId )
        os.makedirs( PointCloudFilePath, exist_ok = True )
        config['DEFAULT']['PointCloudFilePath'] = PointCloudFilePath

        DATABASE = '/data/weight/{0}/'.format( deviceId )
        os.makedirs( DATABASE, exist_ok = True )
        config['DEFAULT']['DATABASE'] = DATABASE + 'weight.db'

        with open( weight_ini, 'w' ) as f:
            config.write( f )
    except Exception as ex:
        print( ex )


if __name__ == '__main__':
    auth = loadDeviceAuth( AUTHCONF )
    deviceId = auth.get( 'device_id' )

    if not deviceId:
        deviceId = uuid.uuid1()
    else:
        updateWeightConf( WEIGHTCONF, deviceId )

    print( 'update weight success,deviceid={0}'.format( deviceId ) )
