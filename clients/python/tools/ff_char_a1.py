"""#1 だけを一定速度で上下に動かし、速度ごと・方向ごとの平均 Duty を測る（FF 校正用）。pitch = roll = x で #1 だけ動く（m1 = 512 + 2g·x, m2 一定）。"""
import json, sys, time, urllib.request
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1]))
from h3_motion_client import MotionClient
B=__import__("os").environ.get("H3CS_URL","http://127.0.0.1:8892"); LOG=open(sys.argv[1],"a",buffering=1)
def post(p,b=None):
    req=urllib.request.Request(B+p,data=json.dumps(b or {}).encode(),headers={"Content-Type":"application/json"},method="POST")
    try: return json.loads(urllib.request.urlopen(req,timeout=3).read())
    except urllib.error.HTTPError as e: return json.loads(e.read() or b"{}")
def st(): return json.load(urllib.request.urlopen(B+"/v1/status",timeout=2))
def ax(s): return {a["axis_id"]:a for a in s["axes"]}
def log(*a): line=time.strftime("%H:%M:%S")+" "+" ".join(str(x) for x in a); print(line); LOG.write(line+"\n")
def bail(m): log("ABORT",m); post("/v1/stop"); sys.exit(1)
s=st(); log("start",s["rig_state"])
if s["rig_state"] not in ("READY_TO_ARM","BRAKED_DISARMED"): bail("not armable")
post("/v1/arm"); t0=time.time()
while time.time()-t0<10 and st()["rig_state"]!="HOLDING": time.sleep(0.1)
post("/v1/center"); t0=time.time()
while time.time()-t0<40 and not (st()["rig_state"]=="HOLDING" and time.time()-t0>1): time.sleep(0.1)
c=MotionClient(B); c.start(); t0=time.time()
while time.time()-t0<10 and not c.connected: time.sleep(0.1)
fs=0
for _ in range(10): fs+=1; c.submit_pose(0,0,0,1.0,frame_seq=fs); time.sleep(0.02)
r=None
for _ in range(30):
    fs+=1; c.submit_pose(0,0,0,1.0,frame_seq=fs); r=post("/v1/start")
    if r.get("ok"): break
    time.sleep(0.3)
log("start ->", r.get("rig_state"), r.get("error")); t0=time.time()
while time.time()-t0<5 and st()["rig_state"]!="ACTIVE": time.sleep(0.1)
if st()["rig_state"]!="ACTIVE": c.close(); bail("not ACTIVE")
# virtual per x: m1 = 512 + 2*511*x ; count ≈ 5.1 count/virtual ; so dm1/dt counts/s = 1022*5.1*dx/dt ≈ 5200*dx/dt
CPX=5200.0  # counts per unit x (approx)
AMP_X=0.35  # ±0.35 → ±1800 counts around neutral (stays inside soft range)
results=[]
for v in (100, 200, 400, 600):
    dxdt=v/CPX
    for direction in (+1, -1):
        # ramp from -AMP to +AMP (or reverse) at constant dx/dt, sampling pwm in the middle 60 % of the ramp
        x0=-AMP_X*direction; x1=AMP_X*direction; dur=(2*AMP_X)/dxdt
        # first go to x0 at 600 count/s (quick), settle 1.5 s
        tstart=time.time(); x=None
        while time.time()-tstart<abs(x0-(x if x is not None else 0))*CPX/600+0.5:
            fs+=1; c.submit_pose(x0,x0,0,1.0,frame_seq=fs); time.sleep(0.02)
        ts=time.time()
        while time.time()-ts<1.5: fs+=1; c.submit_pose(x0,x0,0,1.0,frame_seq=fs); time.sleep(0.02)
        t0=time.time(); samples=[]; vels=[]; errs=[]
        while True:
            t=time.time()-t0
            if t>dur: break
            x=x0+(x1-x0)*t/dur
            fs+=1; c.submit_pose(x,x,0,1.0,frame_seq=fs)
            if 0.2*dur<t<0.8*dur:
                s_=st()
                if s_["rig_state"]!="ACTIVE": c.close(); bail(f"left ACTIVE: {s_['rig_state']} {s_['fault']}")
                a=ax(s_)[1]
                if abs(a["following_error_count"] or 0)>250: c.close(); bail(f"FE on axis1 {a['following_error_count']}")
                samples.append(a["pwm_permille"] or 0); vels.append(a["velocity_count_s"] or 0); errs.append(a["following_error_count"] or 0)
            time.sleep(0.02)
        if samples:
            mp=sum(samples)/len(samples); mv=sum(vels)/len(vels); me=sum(errs)/len(errs)
            nz=[p for p in samples if p]; onfrac=len(nz)/len(samples)
            results.append((v,direction,mp,mv,me,onfrac)); log(f"   (count {ax(st())[1]['actual_count']})")
            log(f"v={v:4d} dir={'UP' if direction>0 else 'DN'} mean_pwm={mp/10:+5.1f}% mean_vel={mv:+6.0f} mean_err={me:+5.0f} duty_on={onfrac*100:3.0f}% n={len(samples)}")
# back to neutral
for _ in range(100): fs+=1; c.submit_pose(0,0,0,1.0,frame_seq=fs); time.sleep(0.05)
post("/v1/stop"); t0=time.time()
while time.time()-t0<20 and st()["rig_state"]!="BRAKED_DISARMED": time.sleep(0.2)
log("end", st()["rig_state"]); c.close()
