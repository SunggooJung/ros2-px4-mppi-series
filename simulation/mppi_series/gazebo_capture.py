"""Record the real Gazebo GUI of the same owned SITL run, never a log redraw."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from .episode_10_1 import write,sha


class GazeboCapture:
    def __init__(self,out,workspace,partition):
        self.out=out;self.processes=[];self.handles=[];self.display=':98'
        if Path('/tmp/.X11-unix/X98').exists():raise RuntimeError('Capture display :98 already in use')
        xvfb='/home/sg/.local/share/stack-series-tools/usr/bin/Xvfb'
        self.launch([xvfb,self.display,'-screen','0','960x720x24','-nolisten','tcp','-ac','+extension','GLX'],os.environ,'capture_xvfb.log')
        deadline=time.monotonic()+10
        while not Path('/tmp/.X11-unix/X98').exists():
            if time.monotonic()>deadline:raise RuntimeError('Capture Xvfb not ready')
            time.sleep(.1)
        (out/'capture_runtime').mkdir(mode=0o700)
        models=workspace/'third_party/px4_sitl/PX4-Autopilot/Tools/simulation/gz/models'
        env={**os.environ,'DISPLAY':self.display,'GZ_PARTITION':partition,
             'QT_QPA_PLATFORM':'xcb','LIBGL_ALWAYS_SOFTWARE':'1','LP_NUM_THREADS':'2','GALLIUM_DRIVER':'llvmpipe',
             'GZ_SIM_RESOURCE_PATH':str(models),'GZ_LOG_PATH':str(out/'capture_gz_logs'),
             'XDG_RUNTIME_DIR':str(out/'capture_runtime')}
        config=out/'capture_gui.config'
        config.write_text('''<?xml version="1.0"?>
<dialog name="quick_start" show_again="false"/>
<window><width>960</width><height>720</height><dialog_on_exit>false</dialog_on_exit>
<menus><drawer default="false"/></menus></window>
<plugin filename="MinimalScene" name="3D View">
<gz-gui><title>Gazebo SITL — MPPI flight</title><property type="bool" key="showTitleBar">false</property><property type="string" key="state">docked</property></gz-gui>
<engine>ogre</engine><scene>scene</scene><ambient_light>0.4 0.4 0.4</ambient_light>
<background_color>0.7 0.7 0.7</background_color><camera_pose>-2 -3 3 0 0.3 1.0</camera_pose><anti_aliasing>1</anti_aliasing>
</plugin>
<plugin filename="GzSceneManager" name="Scene Manager"><gz-gui><property key="state" type="string">floating</property><property key="width" type="double">5</property><property key="height" type="double">5</property><property key="showTitleBar" type="bool">false</property></gz-gui></plugin>
''')
        self.launch(['gz','sim','-g','--render-engine-gui','ogre','--gui-config',str(config)],env,'capture_gui.log')
        self.record_start=time.monotonic()
        self.launch(['ffmpeg','-nostdin','-y','-f','x11grab','-framerate','8','-video_size','960x720',
                     '-i',':98.0','-c:v','libx264','-preset','ultrafast','-crf','23','-threads','1',str(out/'gazebo_screen.mp4')],env,'capture_ffmpeg.log')
        self.metadata={'same_run':True,'source':'Actual Gazebo GUI screen capture attached to owned live server partition',
                       'partition':partition,'capture_start_monotonic_s':self.record_start,
                       'size':[960,720],'capture_fps':8,'hardware_used':False}
        write(out/'screen_capture.json',self.metadata)

    def launch(self,command,env,log):
        handle=(self.out/log).open('w');self.handles.append(handle)
        child=subprocess.Popen(command,env=env,stdout=handle,stderr=subprocess.STDOUT,start_new_session=True)
        self.processes.append(child);return child

    def stop(self):
        # Flush the recorder before stopping the GUI and display.
        for child in reversed(self.processes):
            if child.poll() is None:
                os.killpg(child.pid,signal.SIGINT)
                try:child.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
        for handle in self.handles:handle.close()

    def gif(self,events):
        from PIL import Image
        handover=next(e['monotonic_s'] for e in events if e['event']=='mppi_handover')
        start=max(0.,handover-self.record_start+1.)
        output=self.out/'gazebo_mppi_flight.gif'
        # Skip the first second at handover and last two seconds to leave a
        # margin for recorder startup. Fixed camera, continuous 27s at 3x.
        filters=("setpts=(PTS-STARTPTS)/3,fps=10,scale=800:-1:flags=lanczos,"
                 "drawbox=x=0:y=0:w=iw:h=45:color=black@0.75:t=fill,"
                 "drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf:"
                 "text='Gazebo SITL | MPPI flight | 3x speed':x=18:y=13:fontsize=20:fontcolor=white,"
                 "split[s0][s1];[s0]palettegen=max_colors=128[p];[s1][p]paletteuse=dither=bayer:bayer_scale=5")
        command=['ffmpeg','-nostdin','-y','-ss',str(start),'-t','27','-i',str(self.out/'gazebo_screen.mp4'),
                 '-filter_complex',filters,'-threads','1','-loop','0',str(output)]
        with (self.out/'capture_gif.log').open('w') as log:
            subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
        with Image.open(output) as gif:
            if gif.format!='GIF' or gif.n_frames<50:raise RuntimeError('Animated GIF frames missing')
            hashes=[]
            for i in range(gif.n_frames):
                gif.seek(i);hashes.append(sha_frame(gif.convert('RGB').crop((100,90,700,550))))
            if len(set(hashes))<20:raise RuntimeError('Scene did not visibly change; inspect actual captured frames')
            gif.seek(gif.n_frames//2);gif.convert('RGB').save(self.out/'gif_preview.png')
            self.metadata.update(gif_frames=gif.n_frames,gif_size=list(gif.size),gif_bytes=output.stat().st_size,
                                 gif_sha256=sha(output),clip_start_monotonic_s=self.record_start+start,
                                 clip_source_duration_s=27.,playback_speed=3.,distinct_scene_frames=len(set(hashes)),
                                 gif_command=command)
        write(self.out/'screen_capture.json',self.metadata)
        return output


def sha_frame(image):
    import hashlib
    return hashlib.sha256(image.tobytes()).hexdigest()
