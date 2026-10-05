"""渲染仿真动画和姿态拼图（无界面）：MUJOCO_GL=osmesa python render_sim.py  -> ../../renders/mujoco_*.{gif,png}"""
import os, numpy as np, mujoco
from PIL import Image, ImageDraw, ImageFont
os.chdir(os.path.dirname(os.path.abspath(__file__)))
m=mujoco.MjModel.from_xml_path('scene.xml'); d=mujoco.MjData(m)
KEY={m.key(i).name:m.key_qpos[i][7:].copy() for i in range(m.nkey)}
J=[m.joint(i).name for i in range(1,m.njnt)]
QADR=np.array([m.jnt_qposadr[i] for i in range(1,m.njnt)]); VADR=np.array([m.jnt_dofadr[i] for i in range(1,m.njnt)])
GEAR=m.actuator_gear[:,0].copy()
KP=np.array([20.0 if "neck" in j else (60.0 if "knee" in j else 40.0) for j in J]); KD=np.array([0.3 if "neck" in j else (1.5 if "knee" in j else 1.0) for j in J])
SEQ=[("stand",1.0,"站立"),("look",1.5,"转头抬头"),("stand",1.2,"站立"),("sit",2.0,"坐下"),("stand",2.0,"站起"),("lie",2.5,"趴下"),("crouch",4.0,"收腿蹲伏"),("stand",3.0,"站起")]
W,H=560,350
r=mujoco.Renderer(m,H,W)
cam=mujoco.MjvCamera(); cam.type=mujoco.mjtCamera.mjCAMERA_FREE; cam.distance=1.25; cam.azimuth=140; cam.elevation=-18
opt=mujoco.MjvOption(); opt.geomgroup[3]=0; opt.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER]=0
FONTS = ["/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc", "C:/Windows/Fonts/msyh.ttc", "/System/Library/Fonts/PingFang.ttc"]
fp = next((p for p in FONTS if os.path.exists(p)), None)
font = ImageFont.truetype(fp, 22) if fp else ImageFont.load_default()
mujoco.mj_resetDataKeyframe(m,d,m.key("stand").id); mujoco.mj_forward(m,d)
frames=[]; stills={}
cur=KEY["stand"].copy(); fps=10; every=int(1/(fps*m.opt.timestep))
for name,dur,cn in SEQ:
    n=int((dur+0.8)/m.opt.timestep)
    for k in range(n):
        s=min(1,k*m.opt.timestep/dur); s=s*s*(3-2*s)
        qref=cur+(KEY[name]-cur)*s
        tau=KP*(qref-d.qpos[QADR])-KD*d.qvel[VADR]
        d.ctrl[:]=np.clip(tau/GEAR,m.actuator_ctrlrange[:,0],m.actuator_ctrlrange[:,1])
        mujoco.mj_step(m,d)
        if k%every==0:
            cam.lookat[:]=[d.qpos[0],d.qpos[1],0.2]
            r.update_scene(d,cam,opt); im=Image.fromarray(r.render())
            dr=ImageDraw.Draw(im); dr.text((16,12),f"MuJoCo 仿真 · {cn}",fill=(230,235,240),font=font)
            dr.text((16,H-34),f"t = {d.time:4.1f} s   机身高度 {d.qpos[2]*1000:.0f} mm",fill=(170,178,186),font=font)
            frames.append(im)
    stills[cn]=frames[-1]
    cur=KEY[name].copy()
frames[0].save('../../renders/mujoco_sequence.gif',save_all=True,append_images=frames[1:],duration=int(1000/fps),loop=0,optimize=True)
order=["站立","转头抬头","坐下","趴下","收腿蹲伏","站起"]
sheet=Image.new("RGB",(W*3,H*2),(20,22,26))
for i,k in enumerate(order): sheet.paste(stills[k],((i%3)*W,(i//3)*H))
sheet.save('../../renders/mujoco_poses.png')
print(len(frames), "帧 ->", os.path.abspath("../../renders"))
