'''Useful functions for PyEPICS scripting.

Alan Kastengren, XSD, APS

Started: February 13, 2015
'''
import epics
import numpy as np
import time
import math
import logging
import PyEpics_Utilities as peu

#Add a comment just for fun
SR_current_PV = epics.PV('S:SRcurrentAI.VAL')
A_shutter_closed_PV = epics.PV('PB:07BM:STA_A_FES_CLSD_PL.VAL')
B_shutter_closed_PV = epics.PV('PB:07BM:STA_B_SBS_CLSD_PL.VAL')
threshold_SR_current=30

class AerotechDriver():
    def __init__(self, motor='7bmb1:aero:m1', asynRec='7bmb1:PSOFly1:cmdWriteRead', axis='Z', PSOInput=3,encoder_multiply=1e5):
        self.motor = epics.Motor(motor)
        self.asynRec = epics.PV(asynRec + '.BOUT')
        self.axis = axis
        self.PSOInput = PSOInput
        self.encoder_multiply = encoder_multiply

    def fprogram_PSO(self, fly_scan_pos):
        '''Performs programming of PSO output on the Aerotech driver.
        '''
        #Place the motor at the position where the first PSO pulse should be triggered
        self.motor.move(fly_scan_pos.PSO_positions[0])

        #Make sure the PSO control is off
        self.asynRec.put('PSOCONTROL %s RESET' % self.axis, wait=True, timeout=300.0)
        time.sleep(0.05)
      
        ## initPSO: commands to the Ensemble to control PSO output.
        # Everything but arming and setting the positions for which pulses will occur.
        #Set the output to occur from the I/O terminal on the controller
        self.asynRec.put('PSOOUTPUT %s CONTROL 1' % self.axis, wait=True, timeout=300.0)
        time.sleep(0.05)
        #Set a pulse 10 us long, 20 us total duration, so 10 us on, 10 us off
        self.asynRec.put('PSOPULSE %s TIME 20,10' % self.axis, wait=True, timeout=300.0)
        time.sleep(0.05)
        #Set the pulses to only occur in a specific window
        self.asynRec.put('PSOOUTPUT %s PULSE WINDOW MASK' % self.axis, wait=True, timeout=300.0)
        time.sleep(0.05)
        #Set which encoder we will use.  3 = the MXH (encoder multiplier) input, which is what we generally want
        self.asynRec.put('PSOTRACK %s INPUT %d' % (self.axis, self.PSOInput), wait=True, timeout=300.0)
        time.sleep(0.05)
        #Set the distance between pulses.  Do this in encoder counts.
        self.asynRec.put('PSODISTANCE %s FIXED %d' % (self.axis, fly_scan_pos.delta_encoder_counts), wait=True, timeout=300.0)
        time.sleep(0.05)
        #Which encoder is being used to calculate whether we are in the window.  1 for single axis
        self.asynRec.put('PSOWINDOW %s 1 INPUT %d' % (self.axis, self.PSOInput), wait=True, timeout=300.0)
        time.sleep(0.05)

        #Calculate window function parameters.  Must be in encoder counts, and is 
        #referenced from the stage location where we arm the PSO.  We are at that point now.
        #We want pulses to start at start - delta/2.  
        range_start = -round(fly_scan_pos.delta_encoder_counts / 2) * fly_scan_pos.overall_sense
        range_length = fly_scan_pos.PSO_positions.shape[0] * fly_scan_pos.delta_encoder_counts
        print(range_start, range_length, fly_scan_pos.overall_sense)
        #The start of the PSO window must be < end.  Handle this.
        if fly_scan_pos.overall_sense > 0:
            window_start = range_start
            window_end = window_start + range_length
        else:
            window_end = range_start
            window_start = window_end - range_length
        #Remember, the window settings must be in encoder counts
        self.asynRec.put('PSOWINDOW %s 1 RANGE %d,%d' % (self.axis, window_start-5, window_end+5), wait=True, timeout=300.0)
        print('PSOWINDOW %s 1 RANGE %d,%d' % (self.axis, window_start, window_end))
        #Arm the PSO
        time.sleep(0.05)
        self.asynRec.put('PSOCONTROL %s ARM' % self.axis, wait=True, timeout=300.0)
        #Move to the actual start position and set the motor speed
        self.motor.move(fly_scan_pos.motor_start, wait=True)
        self.motor.put('slew_speed', fly_scan_pos.speed, wait=True)

    def fcleanup_PSO(self):
        '''Cleanup activities after a PSO scan. 
        Turns off PSO and sets the speed back to default.
        '''
        self.asynRec.put('PSOWINDOW %s OFF' % self.axis, wait=True)
        self.asynRec.put('PSOCONTROL %s OFF' % self.axis, wait=True)
        self.motor.put('slew_speed', self.default_speed, wait=True)
 

Aerotech_Y = AerotechDriver(motor='7bmb1:aero:m1', asynRec='7bmb1:PSOFly1:cmdWriteRead', axis='Z', PSOInput=3, encoder_multiply=1e5)

Aerotech_X = AerotechDriver(motor='7bmb1:aero:m2', asynRec='7bmb1:PSOFly2:cmdWriteRead', axis='X', PSOInput=3, encoder_multiply=1e5)

Aerotech_Theta = AerotechDriver(motor='7bmb1:aero:m3', asynRec='7bmb1:PSOFly3:cmdWriteRead', axis='A', PSOInput=3, encoder_multiply=float(2**15)/0.36)

class FlyScanPositions():
    '''This class holds critical parameters for a fly scan.
    Important fields:
    driver: the AerotechDriver object for the driver
    speed: speed during the fly scan
    req_start: the requested start point.  This is the center of the data point
    req_end: the requested end position.  This is the center of the data point.
    req_delta: the requested distance between PSO pulses.
    '''    
    def __init__(self, driver=Aerotech_Y, speed=1, start=0, end=1, delta=0.1):
        self.driver = driver
        self.speed = speed
        self.req_start = start
        self.req_end = end
        self.req_delta = delta
    
    def fcompute_positions_MCS(self):
        '''Computes several parameters describing the fly scan motion.
        These include the actual start position of the motor, the actual 
        distance (in encoder counts and distance) between PSO pulses,
        the end position of the motor, and where PSO pulses are expected to occcur.
        This code ensures that each PSO delta is an integer number of encoder
        counts, since this is how the PSO triggering works in hardware.
        
        These calculations are for MCS scans, where for N bins we need N+1 pulses.

        Several fields are set in the class.
        user_direction: 1 if we are moving + in user coordinates, -1 if -
        overall_sense: is our fly motion + or - with respect to encoder counts
        delta_encoder_counts: integer number of encoder counts per PSO delta
        delta_egu: delta_encoder_counts in engineering units.  May not = delta
        motor_start: where motor will be to start motion.  Accounts for accel distance
        motor_end: where motor will stop motion.  Accounts for decel distance
        actual_end: center of last data point.  May not = req_end due to integer delta_encoder_counts
        PSO_positions: array of places where PSO pulses should occur        
        '''
        # Encoder direction compared to dial coordinates.  Hard code this; could ask controller
        self.encoderDir = -1
        #Get motor direction (dial vs. user)
        motor_dir = self.driver.motor.direction    #0 = positive, 1 = neg
        self.motor_dir = -1 if motor_dir else 1
        #Figure out whether motion is in positive or negative direction in user coordinates
        self.user_direction = 1 if self.req_end > self.req_start else -1
        #Figure out overall sense: +1 if motion in + encoder direction, -1 otherwise
        self.overall_sense = self.user_direction * self.motor_dir * self.encoderDir
        print("Overall sense = " + str(self.overall_sense))
        
        #Get the distance needed for acceleration = 1/2 a t^2 = 1/2 * v * t
        motor_accl_time = self.driver.motor.acceleration    #Acceleration time in s
        accel_dist = motor_accl_time * self.speed / 2.0  

        #Compute the actual delta to keep things at an integral number of encoder counts
        self.delta_encoder_counts = round(self.req_delta * self.driver.encoder_multiply)
        self.delta_egu = self.delta_encoder_counts / self.driver.encoder_multiply
        print("Actual spacing for integer number of encoder counts per bin = {0:f}"
                .format(self.delta_egu))
                    
        #Make taxi distance an integral number of measurement deltas >= accel distance
        #Add 1/2 of a delta, since we want integration centered on start and end.
        taxi_dist = (math.ceil(accel_dist / self.delta_egu) + 0.5) * self.delta_egu
        self.motor_start = self.req_start - taxi_dist * self.user_direction
        self.motor_end = self.req_end + taxi_dist * self.user_direction
        
        #How many points will we have, and where is the last PSO pulse really centered?
        num_points = round(math.fabs(self.req_start-self.req_end) / self.delta_egu) + 2
        self.actual_end = self.req_start + (num_points - 2) * self.delta_egu
        end_spacing = self.delta_egu / 2.0 * self.user_direction
        self.PSO_positions = np.linspace(self.req_start - end_spacing, self.actual_end + end_spacing, num_points)
        print(self)
    
    def fcompute_positions_tomo(self):
        '''Computes several parameters describing the fly scan motion.
        These calculations are for tomography scans, where for N bins we need N pulses.
        See documentation for fcompute_positions_MCS for more description.
        '''
        self.fcompute_positions_MCS()
        self.PSO_positions = self.PSO_positions[:-1]

    
    def fprogram_PSO(self):
        '''Cause the Aerotech driver to program its PSO.
        '''
        self.driver.fprogram_PSO(self)

    def __str__(self):
        return '''
                    Start = {0:f}
                    End = {1:f}
                    Encoder Counts / Pulse = {2:d}
                    EGU / Pulse = {3:f}'''.format(self.req_start, self.actual_end, self.delta_encoder_counts, self.delta_egu)
    
    def fprogram_scan_record(scan_record='7bmb1:scan1', mcs='7bmb1:3820'):
        '''Programs the scan record for use in MCS fly scans.
        Sets the correct number of points, start, end, and scan settings.
        '''
        #Set up the scan record parameters
        #Set the start and end motor positions, as well as the number of points.
        epics.caput(scan+'.P1SP', self.motor_start, wait=True)
        epics.caput(scan+'.P1EP', self.motor_end, wait=True)
        epics.caput(scan+'.NPTS', self.PSO_positions.shape[0] - 1, wait=True)
        #Set the scan record into fly scan mode
        epics.caput(scan+'.P1SM', 2, wait=True, timeout=300.0)
        #Make the only trigger PV the EraseStart PV for the MCS
        epics.caput(scan+'.T1PV', mcs+':EraseStart', wait=True)
        for i in [2,3,4]:
            epics.caput(scan+'.T'+str(i)+'PV', "", wait=True)
        #Set the detectors to 1D array mode
        epics.caput(scan+'.ACQT',1, wait=True, timeout=300.0)

        #Set up the number of points that we will use for the MCS
        epics.caput(mcs+':NuseAll',num_points, wait=True, timeout=300.0)
        #Set up the channel advance to come from an external source, no prescale
        epics.caput(mcs + 'ChannelAdvance', 1, wait=True)
        epics.caput(mcs + 'Prescale', 1, wait=True)
        epics.caput(mcs + 'CountOnStart', 0, wait=True)
        
        #Put the central positions into an array variable we can add as a detector
        epics.caput('7bmb1:userArrayCalc1.AVAL',
                    np.linspace(self.req_start, self.actual_end, self.PSO_positions.shape[0] - 1))

def ftomo_fly_scan_wb(trigger_busy = '7bmb1:busy5', 
                    trigger_PVs={'7bm_pg1:cam1:Acquire':1}, cam_root = '7bm_pg1:'):
    '''Script to perform actions for WB tomography fly scan at 7-BM.
    Script waits for trigger_busy to be triggered.
    The scan then programs the stage for PSO output, checks that motors are
    within limits, takes bright and dark fields, then performs the actual
    tomography scan.  
    This script places the data into a properly formatted DataExchange file
    with the proper meta data.
    The script also monitors the trigger_busy during the scan and aborts if
    it is manually set to zero.
    The script monitors the scan and aborts if the motion stops before acquisition 
    is complete, indicating that the scan has hung.
    
    This script does one scan.  This requires a daemon process if one does not
    want to explicitly call it for every tomography scan.
    '''
    #Input EPICS PVs for scan parameters
    speed_PV = epics.PV('7bmb1:var:float1')
    delta_PV = epics.PV('7bmb1:var:float2')
    start_PV = epics.PV('7bmb1:var:float3')
    end_PV = epics.PV('7bmb1:var:float4')
    retrace_PV = epics.PV('7bmb1:var:float5')
 
    #Input EPICS PVs for bright/dark image parameters
    bright_x_pos = epics.PV('7bmb1:var:float8')
    bright_y_pos = epics.PV('7bmb1:var:float9')
    bd_imgnum = epics.PV('7bmb1:var:int2')
    bright_exp = epics.PV('7bmb1:var:float10')
    #Important variables
    trigger_busy_PV = epics.PV(trigger_busy)
    HDF_capture_PV = epics.PV(cam_root+'HDF1:Capture')
    num_images_PV = epics.PV(cam_root+'cam1:NumImages')
    exposure_time_PV = epics.PV(cam_root+'cam1:AcquireTime')
    sample_x_motor = epics.Motor('7bmb1:aero:m2')
    sample_y_motor = epics.Motor('7bmb1:aero:m1')

    #Make PV objects from the trigger PVs
    trig_pv_dict = {}
    for key in trigger_PVs.keys():
        trig_pv_dict[key] = epics.PV(key)

    counter = 0
    #Have a flag variable to show if scan completed successfully.
    successful_scan = False
    try:
        #Loop to check for starting a scan.
        while True:
            #If we are starting a scan ...
            if trigger_busy_PV.value == 1:
                #Set the retrace speed on the Aerotech driver
                Aerotech_Theta.default_speed = float(retrace_PV.get(wait=True))
                #Clean up the PSO programming in case things are messed up.
                Aerotech_Theta.fcleanup_PSO()
                #Set up the object with fly scan positions
                fly_scan_pos = Fly_Scan_Positions(Aerotech_Theta, speed_PV.value, start_PV.value, end_PV.value, delta_PV.value)
                fly_scan_pos.fcompute_positions_tomo()
                #Check that all positions will be within the motor limits.  If not, throw an exception
                if not (sample_x_motor.within_limits(bright_x_pos) and sample_y_motor.within_limits(bright_y_pos)):
                    print('Bright position not within motor limits.  Check motor limits.')
                    raise ValueError
                if not Aerotech_Theta.motor.within_limits(fly_scan_pos.motor_start):
                    print('Rotation start position not within motor limits.  Check motor limits.')
                    raise ValueError
                if not Aerotech_Theta.motor.within_limits(fly_scan_pos.motor_start):
                    print('Rotation end position not within motor limits.  Check motor limits.')
                    raise ValueError
                #close the shutters 
                peu.fclose_A_shutter()
                print("Grabbing bright and dark fields")
                #Check if the HDF writer is in capture mode.
                if HDF_capture_PV.get() == 1:
                    #Stop the capture.  If we don't here, we will crash the camera soft IOC.
                    HDF_capture_PV.put(0,wait=True)
                #Check if we have any array data.  If not, get it.
                if int(epics.caget(cam_root + 'HDF1:ArraySize0_RBV')) == 0:
                    #Set outselves to internal trigger, single trigger.
                    epics.caput(cam_root + 'cam1:TriggerMode', 0, wait=True)
                    epics.caput(cam_root + 'cam1:ImageMode', 0, wait=True)
                    epics.caput(cam_root + 'cam1:Acquire', 1, wait=True)
                #Set up the HDF plugin to stream images to HDF5.
                bgkd_ims = int(bd_imgnum.value)
                total_imgs = int(bgkd_ims * 2 + fly_scan_pos.PSO_positions.shape[0])
                epics.caput(cam_root+'HDF1:NumCapture', total_imgs, wait=True)
                epics.caput(cam_root+'HDF1:ExtraDimSizeN', total_imgs, wait=True)
                epics.caput(cam_root+'HDF1:FileWriteMode', 2, wait=True)
                epics.caput(cam_root+'HDF1:AutoSave', 1, wait=True)
                #set saving of dark field images
                num_images_PV.put(bgkd_ims, wait=True)
                #Compute the proper frame rate for this exposure time
                data_dark_exp_time = exposure_time_PV.get()
                max_framerate = math.floor(1.0 / data_dark_exp_time)
                epics.caput(cam_root+'cam1:FrameRateValAbs', max_framerate, wait=True)
                #write data to /exchange/data_dark in HDF5 file by passing 1 flag to 'FrameType'
                epics.caput(cam_root+'cam1:FrameType',1)
                #Make sure camera is on internal trigger mode, multiple exposure
                epics.caput(cam_root+'cam1:ImageMode',1,wait=True)
                epics.caput(cam_root+'cam1:TriggerMode',0,wait=True)
                #Start the HDF capture process.  This must run until the file is full.
                epics.caput(cam_root+'HDF1:Capture',1,wait=False)
                time.sleep(0.5)
                #Start dark frame image acquisition
                epics.caput(cam_root+'cam1:Acquire',1,wait=True)
                time.sleep(0.5)

                #Capture bright fields
                peu.fopen_A_shutter()
                #Move to correct x and y position
                origx = sample_x_motor.drive
                origy = sample_y_motor.drive
                sample_x_motor.move(bright_x_pos.get(), wait=True)
                sample_y_motor.move(bright_y_pos.get(), wait=True)
                time.sleep(0.25)
                #Set the desired exposure time
                exposure_time_PV.put(bright_exp.get())
                time.sleep(0.25)
                #write data to /exchange/data_white in HDF5 file by passing 2 flag to 'FrameType'
                epics.caput(cam_root+'cam1:FrameType',2)
                epics.caput(cam_root+'cam1:Acquire',1,wait=True)
                print("Bright and dark fields done. Starting Scan...")

                #move back to previous motor position & exposure time.
                sample_x_motor.move(origx, wait=True)
                sample_y_motor.move(origy, wait=True)
                exposure_time_PV.put(data_dark_exp_time, wait=True)
                #Set up the number of images and put us in multiple exposure mode
                num_images_PV.put(fly_scan_pos.PSO_positions.shape[0], wait=True)
                #Set up the HDF plugin to stream images to HDF5.
                #write data to /exchange/data in HDF5 file by passing 0 flag to 'FrameType'
                epics.caput(trig_root+'cam1:FrameType',0)
                #Make sure camera is on external trigger mode, multiple exposures
                epics.caput(trig_root+'cam1:ImageMode',1,wait=True)
                epics.caput(trig_root+'cam1:TriggerMode',3,wait=True)
                time.sleep(0.5)
                #Program the PSO for this motion
                fly_scan_pos.fprogram_PSO()
                #Compute how long it should take to save data set
                total_time = (end_PV.value - start_PV.value) / speed_PV.value
                print("Scan should take {:5.2f} s.".format(total_time))
                #Trigger the PVs we want to trigger
                for key,value in trigger_PVs.items():
                    trig_pv_dict[key].put(value)
                time.sleep(1.0)
                #Trigger the stage to move
                Aerotech_Theta.motor.move(fly_scan_pos.motor_end)
                start_time = time.time()
                #Now, start looking at whether we've finished or have aborted.
                counter = 0
                time.sleep(1.0)
                while (time.time() - start_time) < total_time * 1.5 + 3.0:
                    counter += 1                
                    if trigger_busy_PV.value == 0:
                        print("Aborting scan.")
                        #Stop the image acquisition
                        epics.caput(cam_root + 'cam1:Acquire',0,wait=True)
                        epics.caput(cam_root + 'HDF1:Capture',0,wait=True)
                        #Stop the motor
                        Aerotech_Theta.motor.put('stop_go', 0, wait=True)
                        time.sleep(0.2)
                        Aerotech_Theta.motor.put('stop_go', 3, wait=True)
                        #Break so we can clean up
                        break
                    #See if all of the triggered PVs are done.  If so, success!
                    trigger_pv_value_sum = 0
                    for key in trigger_PVs.keys():
                        trigger_pv_value_sum += trig_pv_dict[key].value
                    if not trigger_pv_value_sum:
                        print("Finished all triggers.  Successful scan!")
                        successful_scan = True
                        break
                    else:
                        #If the rotation stage is done moving and we're here, the scan has hung.
                        if Aerotech_Theta.motor.get('moving'):
                            print('Rotation stage is done but the camera is still triggering.  Abort.')
                            trigger_busy_PV.put(0, wait=True)
                            time.sleep(0.5)
                            continue
                        if counter % 5 == 0:
                            print("Elapsed time = {:5.2f} s.".format(time.time() - start_time))   
                        time.sleep(1.0)
                else:
                    print("Never finished images in time.  Error!")
                    #Stop the image acquisition
                    epics.caput(cam_root + 'cam1:Acquire',0,wait=True)
                    epics.caput(cam_root + 'HDF1:Capture',0,wait=True)
                #Trigger the cleanup 
                print("Cleaning up the scan.")
                Aerotech_Theta.fcleanup_PSO()
                time.sleep(0.5)
                trigger_busy_PV.put('Done',wait=True)
                time.sleep(0.5)
                #Break out of the while loop: we are done
                break
            time.sleep(0.05)
    finally:
        peu.fclose_A_shutter()
    return successful_scan
  

def ffly_scan_daemon(fly_scan_func = ftomo_fly_scan_wb,
                                trigger_busy='7bmb1:busy5', 
                                repeat_busy='7bmb1:busy4',
                                time_end_start=True):
    '''This is a single daemon process to allow fly scans to run automatically.
    If requested, perform a single scan and listen for more.    
    If requested, perform a repeated scan.
        One busy record tells the code to do repeated scans.
        Another busy record is triggered repeatedly from this.
        The variable time_end_start details whether the time
        between scans is really between scans (True) or start-to-start (False).
    '''
    repeated_scan_busy_PV = epics.PV(repeat_busy)
    trigger_busy_PV = epics.PV(trigger_busy)
    counter = 0
    while True:
        #If we see the trigger_busy, this means we are doing one scan.
        if trigger_busy_PV.value == 1:
            fly_scan_func()
        #If we see the repeat_busy, that means to multiple scans.
        if repeated_scan_busy_PV.value == 1:
            num_repeats = int(epics.caget('7bmb1:var:int1'))
            sec_between_pts = float(epics.caget('7bmb1:var:float6'))
            for i in range(num_repeats):
                start_time = time.time()
                if repeated_scan_busy_PV.value == 0:
                    break
                print("In the repeat loop on scan #{:3d}".format(i))
                #Monitor for good beam
                fcheck_for_bad_beam() 
                #Start the scan
                successful_scan = fly_scan_func() 
                #If the scan was bad, abort.
                if not successful_scan:
                    print("Aborting repeated scan: bad individual fly scan encountered.")
                    repeated_scan_busy_PV.value = 0
                break 
                #When scan is done, record the time
                scan_end_time = time.time()
                sec_wait = 0 
                if time_end_start:
                    sec_wait = scan_end_time - start_time
                else:
                    sec_wait = sec_between_pts - scan_end_time + start_time
                print('Scan #{0:d} done.  Waiting {1:d} seconds for next scan.')
                if sec_wait < 1:
                    print('Already late for the next scan.  Start now.')
                    continue 
                for __ in range(int(sec_wait)):
                    #Check to make sure we haven't clicked "Done" on repeated_scan_busy to abort this.
                    print("Waiting for the next scan.")                        
                    if repeated_scan_busy_PV.value == 0:
                        print("Scan aborted!")
                        break
                    time.sleep(1.0)
            repeated_scan_busy_PV.value = 0
            time.sleep(0.5)
        if counter == 100:
            print("Looking for repeated scan busy at time " + time.strftime('%H:%M:%S',time.localtime()))
            counter = 0
        else:
            counter += 1
        time.sleep(0.01)

def ffly_scan_daemon_tomo(trigger_busy='7bmb1:busy5', 
                                repeat_busy='7bmb1:busy4',
                                time_end_start=True):
    return fauto_repeated_fly(ftomo_fly_scan_wb, trigger_busy, repeat_busy, time_end_start)

def ffly_scan_daemon_mcs(trigger_busy='7bmb1:busy5', 
                                repeat_busy='7bmb1:busy4',
                                time_end_start=True):
    return fauto_repeated_fly(fmcs_fly_scan, trigger_busy, repeat_busy, time_end_start)

 
def fMCS_fly_scan(trigger_busy = '7bmb1:busy5', scan_record='7bmb1:scan1',
                    driver=Aerotech_Y, mcs='7bmb1:3820'):
    '''Script to perform actions for MCS fly scans at 7-BM.
    Script waits for trigger_busy to be triggered.
    The scan then programs the stage for PSO output, checks that scan motor is
    within limits, programs the scan record, then performs the fly scan.  
    The script also monitors the trigger_busy during the scan and aborts if
    it is manually set to zero.
    The script monitors the scan and aborts if the motion stops before acquisition 
    is complete, indicating that the scan has hung.
    
    This script does one scan.  This requires a daemon process if one does not
    want to explicitly call it for every fly scan.
    '''
    #Input EPICS PVs for scan parameters
    speed_PV = epics.PV('7bmb1:var:float1')
    delta_PV = epics.PV('7bmb1:var:float2')
    start_PV = epics.PV('7bmb1:var:float3')
    end_PV = epics.PV('7bmb1:var:float4')
    retrace_PV = epics.PV('7bmb1:var:float5')
    
    counter = 0
    #Have a flag variable to show if scan completed successfully.
    successful_scan = False
    try:
        #Loop to check for starting a scan.
        while True:
            #If we are starting a scan ...
            if trigger_busy_PV.value == 1:
                #Set the retrace speed on the Aerotech driver
                driver.default_speed = float(retrace_PV.get(wait=True))
                #Clean up the PSO programming in case things are messed up.
                driver.fcleanup_PSO()
                #Set up the object with fly scan positions
                fly_scan_pos = Fly_Scan_Positions(driver, speed_PV.value, 
                                    start_PV.value, end_PV.value, delta_PV.value)
                fly_scan_pos.fcompute_positions_mcs()
                fly_scan_pos.fprogram_scan_record(scan_record, mcs)
                #Check that all positions will be within the motor limits.  If not, throw an exception
                if not driver.motor.within_limits(fly_scan_pos.motor_start):
                    print('Scan start position not within motor limits.  Check motor limits.')
                    raise ValueError
                if not driver.motor.within_limits(fly_scan_pos.motor_end):
                    print('Scan end position not within motor limits.  Check motor limits.')
                    raise ValueError
                #Program the PSO for this motion
                fly_scan_pos.fprogram_PSO()
                #Open the shutters
                peu.fopen_shutters()
                #Compute how long it should take to save data set
                total_time = (end_PV.value - start_PV.value) / speed_PV.value
                print("Scan should take {:5.2f} s.".format(total_time))
                #Trigger the scan
                epics.caput(scan_record + '.EXSC', wait=False)
                start_time = time.time()
                #Now, start looking at whether we've finished or have aborted.
                counter = 0
                time.sleep(1.0)
                while (time.time() - start_time) < total_time * 1.5 + 3.0:
                    counter += 1                
                    if trigger_busy_PV.value == 0:
                        print("Aborting scan.")
                        #Stop the MCS acquisition
                        epics.caput(mcs + 'StopAll', 1, wait=True)
                        #Abort the scan
                        epics.caput('7bmb1:AbortScans.PROC', 1, wait=False)
                        #Stop the motor
                        Aerotech_Theta.motor.put('stop_go', 0, wait=True)
                        time.sleep(0.2)
                        Aerotech_Theta.motor.put('stop_go', 3, wait=True)
                        #Break so we can clean up
                        break
                    #Check if all of the triggered actions are complete
                    if not epics.caget(scan_record + '.BUSY', wait=True):
                        print("Finished scan.")
                        successful_scan = True
                        break
                    else:
                        #If the stage is done moving and we're here, the scan has hung.
                        if driver.motor.get('moving'):
                            print('Stage is done but data acquisition is not.  Scan hung.  Abort.')
                            trigger_busy_PV.put(0, wait=True)
                            time.sleep(0.5)
                            continue
                        if counter % 5 == 0:
                            print("Elapsed time = {:5.2f} s.".format(time.time() - start_time))   
                        time.sleep(1.0)
                else:
                    print("Scan timeout.  Error!")
                    #Stop the MCS acquisition
                    epics.caput(mcs + 'StopAll', 1, wait=True)
                    #Abort the scan
                    epics.caput('7bmb1:AbortScans.PROC', 1, wait=False)
                #Trigger the cleanup 
                print("Cleaning up the scan.")
                driver.fcleanup_PSO()
                time.sleep(0.5)
                print(trigger_busy_PV.value)
                trigger_busy_PV.put('Done',wait=True)
                time.sleep(0.5)
                print(trigger_busy_PV.value)
            time.sleep(0.05)
    finally:
        peu.fclose_B_shutter()
    return successful_scan

