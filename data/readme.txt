TR_NASA
ENV_NASA

data1.bag - one tracker, two lighthouses, static on the center of granite table
data2.bag - one tracker, two lighthouses, static on dock
data3.bag - one tracker, two lighthouses, static for 30 secs, move for 60 secs guided manuall
y, static for 30 secs

data4.bag - two trackers, two lighthouses, static on center of granite table
data5.bag - two trackers, two lighthouses, static on dock
data6.bag - two trackers, two lighthouses, static for 30 secs, move for 60 secs guided manually, static for 30 secs

data7.bag - two trackers, one lighthouse, static on center of granite table
data8.bag - two trackers, one lighthouse, static on dock
data9.bag - two trackers, one lighthouse, static for 30 secs, move for 60 secs guided manually, static for 30 secs

The following 3 bags do not converge with the regular calibration
data10.bag - one tracker, one lighthouse, static on center of granite table
data11.bag - one tracker, one lighthouse, static on dock
data12.bag - one tracker, one lighthouse, static for 30 secs, move for 30 secs guided manually, static for 30 secs

data13.bag - one tracker, two lighthouses, static on center of granite table, light ON

14, 15 and 16 do not exist
data14.bag - one tracker, two lighthouses, static on center of granite table, light OFF
data15.bag - one tracker, two lighthouses, static for 10 secs, move for 60 secs guided manually, static for 30 secs, light ON
data16.bag - one tracker, two lighthouses, static for 10 secs, move for 60 secs guided manually, static for 30 secs, light OFF

data20.bag - one tracker, two lighthouses, moving back and forward


TR1
ENV1

vivebag_2019-02-11-15-45-40.bag - contains data of the tracker in a static state, using two lighthouses on tripods doing approximately a 90deg angle between their FOV. modes b and c. See attached picture.
For visual support see IMG_T1.jpg and IMG_ENV1.jpg

vivebag_2019-02-11-16-02-12.bag - contains data of the tracker initially in a static state using two lighthouses on tripods. After 30 seconds we start moving the trackers trying to copy the movement of astrobee, but slightly faster. The environment setup was the same as before. The orientation of the tracker didn't change very much during these.

vivebag_2019-02-11-16-07-37.bag - same but mostly the orientation changed during while the data was recorded.

vivebag_2019-02-11-16-10-06.bag - both linear movement and angular movement with the same environment.

ENV2

vivebag_2019-02-11-16-16-56.bag - New environment (see picture IMG_ENV2.jpg). For this dataset, the tracker was stationary in the sofa. // this data is not good - can't calibrate

vivebag_2019-02-11-16-20-15.bag - same environment, but moved the tracker in circles around myself.

vivebag_2019-02-11-16-30-50.bag - the same but not just circles

vivebag_2019-02-11-16-32-48.bag - the same but now moved around the sofa instead of being always standing in the same place

ENV3

vivebeta1.bag - tilted the controllers up and down in the same position

vivebeta2.bag - the same but sideways

vivebeta3.bag - moved the tracker further aways and closer to the lighthouse

vivebeta4.bag - rotated the tracker around sensor 6's normals vector

vivebeta5.bag - tried to collect a bag with all positions and orientations to train model of the beta angle

vivebeta6.bag - big set of data collected with two lighthouses and moving the tracker arround the MoCap System

vivebeta7.bag - smaller set of the same environment (for testing)

ENV4

dat4.1.bag - tracker on the sofa with two lighthouses pointing at it. Static dataset

dat4.2.bag - the same but moved the tracket a bit to the right

dat4.3.bag - the same but to the left

dat4.4.bag - the same but on the floor

dat4.5.bag - on top of chair

dat4.6.bag - on top of the chair but slightly rotated

dat4.7.bag - the same

dat4.8.bag - moving bag

dat4.9.bag - moving bag

dat4.10.bag - moving bag

dat4.11.bag - moving bag

dat4.12.bag - moving bag

ENV5

env5_data1.bag - static on chair

env5_data2.bag - static on floor pillow

env5_data3.bag - same

env5_data4.bag - same

env5_data6.bag - same

env5_data7.bag - same

env5_data8.bag - moving

env5_data9.bag - same

ENV6

data6.1.bag - static on sofa

data6.2.bag - static on sofa, same orientation different position

data6.3.bag - same

data6.4.bag - on sofa again but different orientation

data6.5.bag - same

data6.6.bag - on top of pillow on top os sofa

data6.7.bag - on top of chair

data6.8.bag - same

data6.9.bag - same

data6.10.bag - movement

data6.11.bag - movement - best bag

data6.12.bag - movement

data6.13.bag - movement - worse bag

ENV7

data7.1.bag - data7.8.bag - static

data7.9.bag - data7.11.bag - dynamic

ENV8 - New alpha angle

data8.1.bag - data8.10.bag - static

data8.11.bag - data8.17.bag - dynamic

data8.11.bag & data8.16.bag & data8.17.bag - loops

ENV9 - New alpha angle

data9.1.bag - data9.10.bag - static

data9.11.bag - data9.17.bag - dynamic

Note: the new angle alpha seams to be wrong

ENV10 - Old alpha angle

data10.1.bag - data10.8.bag - static

data10.9.bag - data10.13.bag - dynamic
