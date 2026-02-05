#!/usr/bin/env python3
import json, math, sys
from pathlib import Path

PAIRS = Path(__file__).resolve().parents[1] / "slam_web" / "missions" / "slam_airsim_pairs.json"

def _mean(xs):
    return sum(xs)/max(1,len(xs))

class Pair:
    def __init__(self, sx, sy, sz, ax, ay, ax_body=None, ay_body=None, yaw_deg=None):
        self.sx=sx; self.sy=sy; self.sz=sz; self.ax=ax; self.ay=ay
        self.ax_body=ax_body; self.ay_body=ay_body; self.yaw_deg=yaw_deg


def load_pairs(path):
    j=json.loads(open(path).read())
    out=[]
    for p in j.get('pairs',[]):
        out.append(Pair(sx=float(p['sx']), sy=float(p.get('sy',0.0)), sz=float(p['sz']), ax=float(p['ax']), ay=float(p['ay']), ax_body=float(p.get('ax_body')) if p.get('ax_body') is not None else None, ay_body=float(p.get('ay_body')) if p.get('ay_body') is not None else None, yaw_deg=float(p.get('yaw_deg')) if p.get('yaw_deg') is not None else None))
    return out


def _u_of(p, axes):
    a0=axes[0]
    return p.sx if a0=='x' else (p.sy if a0=='y' else p.sz)

def _v_of(p, axes):
    a1=axes[1]
    return p.sx if a1=='x' else (p.sy if a1=='y' else p.sz)


def solve_transform(pairs, allow_scale, axes):
    if len(pairs)<2:
        raise ValueError('need >=2')
    su=[_u_of(p,axes) for p in pairs]
    sv=[_v_of(p,axes) for p in pairs]
    axs=[p.ax for p in pairs]
    ays=[p.ay for p in pairs]
    msu=_mean(su); msv=_mean(sv); maxx=_mean(axs); mayy=_mean(ays)
    xs=[(_u_of(p,axes)-msu, _v_of(p,axes)-msv) for p in pairs]
    ya=[(p.ax-maxx, p.ay-mayy) for p in pairs]
    h11=sum(x[0]*y[0] for x,y in zip(xs,ya))
    h12=sum(x[0]*y[1] for x,y in zip(xs,ya))
    h21=sum(x[1]*y[0] for x,y in zip(xs,ya))
    h22=sum(x[1]*y[1] for x,y in zip(xs,ya))
    a=(h11+h22); b=(h21-h12)
    denom=math.sqrt(a*a + b*b)
    if denom<=1e-12:
        raise ValueError('degenerate')
    c=a/denom; s=b/denom
    r11,r12 = c, -s
    r21,r22 = s, c
    scale=1.0; model='rigid'
    if allow_scale:
        var_x = sum(x[0]*x[0] + x[1]*x[1] for x in xs)
        if var_x>1e-12:
            scale = denom / var_x
            model='similarity'
    tx = maxx - scale * (r11*msu + r12*msv)
    ty = mayy - scale * (r21*msu + r22*msv)
    return dict(scale=scale, r11=r11,r12=r12,r21=r21,r22=r22, tx=tx, ty=ty, model=model, slam_axes=axes)


def apply_xyz(t, sx, sy, sz):
    u = sx if t['slam_axes'][0]=='x' else (sy if t['slam_axes'][0]=='y' else sz)
    v = sx if t['slam_axes'][1]=='x' else (sy if t['slam_axes'][1]=='y' else sz)
    x = t['scale'] * (t['r11']*u + t['r12']*v) + t['tx']
    y = t['scale'] * (t['r21']*u + t['r22']*v) + t['ty']
    return x,y


def rmse_m(t, pairs):
    if not pairs: return 0.0
    err2=0.0
    for p in pairs:
        x,y = apply_xyz(t,p.sx,p.sy,p.sz)
        dx=x-p.ax; dy=y-p.ay
        err2 += dx*dx + dy*dy
    return math.sqrt(err2/len(pairs))


def airsim_body_to_camera_xy_with_offset(ax, ay, yaw_deg, ox, oy):
    a = math.radians(float(yaw_deg))
    oxr = float(ox)*math.cos(a) - float(oy)*math.sin(a)
    oyr = float(ox)*math.sin(a) + float(oy)*math.cos(a)
    return float(ax)+oxr, float(ay)+oyr


def span_m(pairs):
    # approximate airsim span on XY
    xs=[p.ax for p in pairs]
    ys=[p.ay for p in pairs]
    minx=min(xs); maxx=max(xs); miny=min(ys); maxy=max(ys)
    return math.hypot(maxx-minx, maxy-miny)


def slam_span_3d(pairs):
    # max pairwise distance in SLAM 3D
    pts=[(p.sx,p.sy,p.sz) for p in pairs]
    m=0.0
    for i in range(len(pts)):
        for j in range(i+1,len(pts)):
            dx=pts[i][0]-pts[j][0]; dy=pts[i][1]-pts[j][1]; dz=pts[i][2]-pts[j][2]
            d=math.sqrt(dx*dx+dy*dy+dz*dz)
            if d>m: m=d
    return m


def main():
    pairs = load_pairs(PAIRS)
    if not pairs:
        print('No pairs found at', PAIRS); sys.exit(1)
    print('Loaded', len(pairs), 'pairs')
    ox_b=0.25; oy_b=0.0
    axes_list=[('x','y'),('x','z'),('y','z')]
    # recompute pairs with offset on (using ax_body) and offset off
    pairs_on=[]; pairs_off=[]
    for p in pairs:
        if p.ax_body is None or p.ay_body is None or p.yaw_deg is None:
            # if missing, fall back to stored ax,ay
            pairs_on.append(p)
            pairs_off.append(p)
            continue
        ax_on, ay_on = airsim_body_to_camera_xy_with_offset(p.ax_body, p.ay_body, p.yaw_deg, ox_b, oy_b)
        pairs_on.append(Pair(p.sx,p.sy,p.sz, ax_on, ay_on, p.ax_body, p.ay_body, p.yaw_deg))
        ax_off, ay_off = airsim_body_to_camera_xy_with_offset(p.ax_body, p.ay_body, p.yaw_deg, 0.0, 0.0)
        pairs_off.append(Pair(p.sx,p.sy,p.sz, ax_off, ay_off, p.ax_body, p.ay_body, p.yaw_deg))
    print('\nUsing camera offset ox,oy =', ox_b, oy_b)
    for axes in axes_list:
        try:
            t_on = solve_transform(pairs_on, allow_scale=False, axes=axes)
            e_on = rmse_m(t_on, pairs_on)
            t_off = solve_transform(pairs_off, allow_scale=False, axes=axes)
            e_off = rmse_m(t_off, pairs_off)
            print(f'axes={axes}: rmse_offset_on={e_on:.3f} m, rmse_offset_off={e_off:.3f} m')
        except Exception as ex:
            print('axes=',axes,'failed:', ex)
    # also try best axes (like solve_best_transform)
    print('\nPer-axis best (choose lowest RMSE using stored ax,ay):')
    best=None; best_e=1e18; best_axes=None
    for axes in axes_list:
        try:
            t=solve_transform(pairs, allow_scale=False, axes=axes)
            e=rmse_m(t,pairs)
            print('axes',axes,'rmse using stored ax,ay=',e)
            if e<best_e:
                best_e=e; best=t; best_axes=axes
        except Exception:
            pass
    print('Best axes by stored ax,ay:', best_axes, 'rmse=', best_e)
    print('\nAirsim span_m=', span_m(pairs), 'slam_span_3d_m=', slam_span_3d(pairs))

if __name__=='__main__':
    main()
