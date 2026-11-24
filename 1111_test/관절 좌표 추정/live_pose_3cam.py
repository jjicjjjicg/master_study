"""RealSense RGB preview with 12-keypoint YOLO Pose. Q/Esc: quit, S: snapshot."""
import argparse
from datetime import datetime
from pathlib import Path
import time
import uuid

HERE = Path(__file__).resolve().parent
DEFAULT_MODEL = HERE / 'best.pt'
if not DEFAULT_MODEL.exists():
    DEFAULT_MODEL = HERE.parent / 'outputs/pose/session3_plus_session1_20261027_235755_f16724/training/weights/best.pt'


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--weights', type=Path, default=DEFAULT_MODEL)
    p.add_argument('--list-cameras', action='store_true')
    p.add_argument('--serials', nargs='+', help='Camera serials, in display order (max 3)')
    p.add_argument('--labels', nargs='+', help='ASCII labels in the same order, e.g. top right left')
    p.add_argument('--device', default='auto', help='auto, cpu, or CUDA index such as 0')
    p.add_argument('--conf', type=float, default=0.25, help='Hand detection threshold')
    p.add_argument('--kpt-conf', type=float, default=0.5, help='Keypoint display threshold')
    p.add_argument('--imgsz', type=int, default=640)
    p.add_argument('--fps', type=int, default=15, help='Requested camera FPS, not inference FPS')
    p.add_argument('--separate', action='store_true', help='One window per camera instead of a 2x2 grid')
    p.add_argument('--output', type=Path, default=HERE / 'snapshots')
    p.add_argument('--test-image', type=Path, help='Camera-free inference check; writes one annotated JPG')
    a = p.parse_args()
    if not 0 <= a.conf <= 1 or not 0 <= a.kpt_conf <= 1: p.error('Thresholds must be in [0, 1]')
    if a.imgsz < 32 or a.fps < 1: p.error('imgsz and fps must be positive')
    if a.serials and (len(a.serials)>3 or len(set(a.serials))!=len(a.serials)): p.error('Use 1-3 unique serials')
    return a


def draw_result(frame, result, threshold):
    import cv2
    import numpy as np
    canvas = frame.copy()
    count = 0
    if result.boxes is None or not len(result.boxes): return canvas, count
    # Dataset contains one glove: display the highest-confidence class-0 hand.
    candidates = [i for i,c in enumerate(result.boxes.cls.cpu().numpy()) if int(c)==0]
    if not candidates: return canvas, count
    i = max(candidates, key=lambda j: float(result.boxes.conf[j]))
    x1,y1,x2,y2 = result.boxes.xyxy[i].cpu().numpy().astype(int)
    cv2.rectangle(canvas,(x1,y1),(x2,y2),(255,180,0),1)
    cv2.putText(canvas,f'hand {float(result.boxes.conf[i]):.2f}',(x1,max(55,y1-6)),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,180,0),1)
    if result.keypoints is None: return canvas, count
    points = result.keypoints.xy[i].cpu().numpy()
    confidence = result.keypoints.conf
    confidence = confidence[i].cpu().numpy() if confidence is not None else np.ones(len(points))
    if len(points)!=12: raise ValueError(f'Expected 12 keypoints, got {len(points)}')
    h,w = canvas.shape[:2]
    for j,((x,y),score) in enumerate(zip(points,confidence)):
        if not np.isfinite([x,y,score]).all() or score<threshold or (x==0 and y==0): continue
        if not (0<=x<w and 0<=y<h): continue
        xy=(int(round(x)),int(round(y)))
        cv2.circle(canvas,xy,4,(0,255,255),-1)
        cv2.putText(canvas,f'M{j+1:02} {score:.2f}',(xy[0]+5,xy[1]-5),cv2.FONT_HERSHEY_SIMPLEX,.35,(0,255,255),1)
        count+=1
    return canvas,count


def save_image(path, frame):
    import cv2
    path.parent.mkdir(parents=True,exist_ok=True)
    ok, encoded = cv2.imencode(path.suffix,frame)
    if not ok: raise RuntimeError(f'Cannot encode {path}')
    encoded.tofile(str(path))  # Handles Windows Unicode paths.


def main():
    args=parse_args()
    import cv2
    import numpy as np
    # Camera listing works without loading a model or importing PyTorch.
    if not args.test_image:
        import pyrealsense2 as rs
        context=rs.context()
        devices=sorted((d.get_info(rs.camera_info.serial_number),d.get_info(rs.camera_info.name)) for d in context.query_devices() if 'Platform Camera' not in d.get_info(rs.camera_info.name))
        for serial,name in devices: print(f'{serial}: {name}')
        if args.list_cameras:
            if not devices: print('No RealSense cameras found.')
            return
        if not devices: raise SystemExit('No RealSense cameras found. Check USB and close RealSense Viewer.')
        available={s for s,_ in devices}
        serials=args.serials or [s for s,_ in devices[:3]]
        if not set(serials)<=available: raise SystemExit(f'Camera serials not connected: {set(serials)-available}')
        labels=args.labels or [f'camera{i+1}' for i in range(len(serials))]
        if len(labels)!=len(serials): raise SystemExit('--labels must have one label per selected camera')
    import torch
    from ultralytics import YOLO
    if not args.weights.is_file(): raise SystemExit(f'Model not found: {args.weights}')
    device=('0' if torch.cuda.is_available() else 'cpu') if args.device=='auto' else args.device
    print('Weights:',args.weights.resolve())
    print('Device:',device,'| Q/Esc: quit | S: save current preview')
    model=YOLO(str(args.weights))
    shape=model.model.yaml.get('kpt_shape')
    if list(shape or [])!=[12,3]: raise SystemExit(f'Expected 12-point pose model, got {shape}')
    def infer(images):
        return model.predict(source=images,imgsz=args.imgsz,conf=args.conf,iou=0.7,device=device,verbose=False,save=False)
    if args.test_image:
        frame=cv2.imdecode(np.fromfile(str(args.test_image),dtype=np.uint8),cv2.IMREAD_COLOR)
        if frame is None: raise SystemExit(f'Cannot read {args.test_image}')
        rendered,count=draw_result(frame,infer([frame])[0],args.kpt_conf)
        dest=args.output/('test_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]+'.jpg')
        save_image(dest,rendered)
        print(f'Test OK: {count}/12 displayed, saved {dest}')
        return
    # Warm up before starting cameras. RGB-only keeps USB bandwidth lower.
    infer([np.zeros((480,640,3),dtype=np.uint8)])
    cameras=[]
    try:
        for serial,label in zip(serials,labels):
            pipeline=rs.pipeline(context)
            config=rs.config();config.enable_device(serial)
            config.enable_stream(rs.stream.color,640,480,rs.format.bgr8,args.fps)
            try:
                profile=pipeline.start(config)
                sensor=profile.get_device().first_color_sensor()
                if sensor.supports(rs.option.frames_queue_size):
                    try: sensor.set_option(rs.option.frames_queue_size,1)
                    except RuntimeError: pass
            except RuntimeError as e:
                try: pipeline.stop()
                except RuntimeError: pass
                print(f'Cannot start {label} ({serial}): {e}')
                continue
            cameras.append({'pipeline':pipeline,'serial':serial,'label':label,'last':np.zeros((480,640,3),np.uint8),'time':None,'count':0,'rate':0.0,'last_error':None})
            print(f'Started {label}: {serial}')
        if not cameras: raise SystemExit('No camera streams started.')
        windows=[f"{c['label']} | {c['serial']}" for c in cameras] if args.separate else ['RealSense - latest pose model']
        for name in windows:cv2.namedWindow(name,cv2.WINDOW_NORMAL)
        preview=[]
        while True:
            fresh=[];indices=[];times=[]
            for i,c in enumerate(cameras):
                try:
                    frames=c['pipeline'].poll_for_frames()
                    frame=frames.get_color_frame()
                    if frame:
                        fresh.append(np.asanyarray(frame.get_data()).copy());indices.append(i);times.append(time.monotonic())
                except RuntimeError as e:
                    if c['last_error']!=str(e):print(c['label'],'stream error:',e);c['last_error']=str(e)
            if fresh:
                start=time.monotonic();results=infer(fresh);elapsed=time.monotonic()-start
                for image,result,i,received in zip(fresh,results,indices,times):
                    c=cameras[i]
                    c['rate']=1/(received-c['time']) if c['time'] is not None and received>c['time'] else 0.0
                    c['last'],c['count']=draw_result(image,result,args.kpt_conf)
                    c['time']=received
            preview=[]
            for c in cameras:
                image=c['last'].copy()
                age=time.monotonic()-c['time'] if c['time'] is not None else None
                cv2.rectangle(image,(0,0),(640,48),(0,0,0),-1)
                cv2.putText(image,f"{c['label']} | SN {c['serial']} | {c['count']}/12",(8,18),cv2.FONT_HERSHEY_SIMPLEX,.48,(255,255,255),1)
                status='WAITING FOR RGB' if age is None else f"updates {c['rate']:.1f}/s | frame age {age:.2f}s"
                if age is not None and age>1:status='STALE / NO NEW RGB | '+status
                cv2.putText(image,status,(8,38),cv2.FONT_HERSHEY_SIMPLEX,.43,(0,0,255) if age is None or age>1 else (0,255,0),1)
                preview.append(image)
            if args.separate:
                for name,image in zip(windows,preview):cv2.imshow(name,image)
            else:
                tiles=preview.copy()
                while len(tiles)<4:tiles.append(np.zeros((480,640,3),np.uint8))
                grid=np.vstack((np.hstack(tiles[:2]),np.hstack(tiles[2:4])))
                cv2.imshow(windows[0],grid)
            key=cv2.waitKey(1)&0xFF
            if key in (ord('q'),27):break
            if any(cv2.getWindowProperty(name,cv2.WND_PROP_VISIBLE)<1 for name in windows):break
            if key==ord('s'):
                folder=args.output/(datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6])
                for c,image in zip(cameras,preview):save_image(folder/(c['serial']+'.jpg'),image)
                print('Snapshot saved:',folder)
            if not fresh:time.sleep(.003)
    finally:
        for c in cameras:
            try:c['pipeline'].stop()
            except RuntimeError:pass
        cv2.destroyAllWindows()

if __name__=='__main__':
    main()
