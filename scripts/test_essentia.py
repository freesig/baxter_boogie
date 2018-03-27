import sys
import essentia

import ros
from std_msgs.msg import Float64 

import time

import essentia.standard
import essentia.streaming

from essentia.streaming import VectorInput
from essentia.streaming import RhythmExtractor2013
from essentia.standard import FrameGenerator 

def setup_ros():
    pub = rospy.Publisher('dance', Float64, queue_size=10)
    rospy.init_node('dance_detect', anonymous=True)
    return pub

def track():
    if len(sys.argv) < 2:
        print("Supports 16 bit wave file. Use %s filename.wav" % sys.argv[0])
        sys.exit(-1)

    pub = setup_ros()

    loader = essentia.standard.MonoLoader(filename = sys.argv[1])()

    for frame in FrameGenerator(loader, frameSize = 100*1024, hopSize = 512, startFromZero=True):
        v_in = VectorInput(frame)
        beat_tracker = RhythmExtractor2013(method="degara")
        pool = essentia.Pool()
        v_in.data >> beat_tracker.signal
        beat_tracker.ticks >> (pool, 'Rhythm.ticks')
        beat_tracker.bpm >> (pool, 'Rhythm.bpm')
        beat_tracker.confidence >> None 
        beat_tracker.estimates >> None 
        beat_tracker.bpmIntervals >> None 
        essentia.run(v_in)

        bpm = pool['Rhythm.bpm']
        bps = bpm / 60.0

        next_beat = time.clock() + bps
        pub.publish(next_beat)

if __name__ == '__main__':
    try:
        track()
    except rospy.ROSInterruptException:
        pass
