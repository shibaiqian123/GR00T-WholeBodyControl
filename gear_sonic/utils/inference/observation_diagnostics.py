"""Explain existing freshness gates without renewing timestamps or relaxing limits."""
import math
import numpy as np


def number(x):
    try:
        x=float(x)
        return x if math.isfinite(x) else None
    except (ValueError,TypeError,OverflowError):return None


def inspect_observation(camera,body,dex,limits,now,wall,body_error=''):
    report={'reasons':[], 'body_error':body_error}
    reasons=report['reasons']
    def age(name,stamp,clock=now):
        value=number(stamp)
        result=None if value is None else clock-value
        report[name]=result
        if result is None:reasons.append(name+':missing_or_nonfinite')
        elif not 0<=result<=.25:reasons.append(name+':outside_0_250ms')
        return value
    camera=camera if isinstance(camera,dict) else {}
    body=body if isinstance(body,dict) else {}
    dex=dex if isinstance(dex,dict) else {}
    ts=camera.get('timestamps') or {}
    camera_stamp=age('camera_age_seconds',ts.get('ego_view'),wall)
    observed=None if camera_stamp is None else now-(wall-camera_stamp)
    body_stamp=age('body_age_seconds',body.get('sample_time'))
    report['body_source_tick']=body.get('source_tick')
    age('dex_packet_age_seconds',dex.get('time'))
    stamps=[observed,body_stamp]
    for side in ('left','right'):
        x=(dex.get('sides') or {}).get(side) or {}
        stamps.append(age(side+'_feedback_age_seconds',x.get('feedback_time')))
        report[side+'_feedback_valid']=x.get('feedback_valid') is True
        report[side+'_q']=number(x.get('q'))
        if x.get('feedback_valid') is not True:reasons.append(side+':feedback_invalid')
        q=report[side+'_q']
        if q is None or not limits[side].contains(q):reasons.append(side+':position_invalid_or_outside_limits')
    report['observation_skew_seconds']=max(stamps)-min(stamps) if all(x is not None for x in stamps) else None
    if report['observation_skew_seconds'] is not None and report['observation_skew_seconds']>.15:reasons.append('observation_skew:over_150ms')
    image=np.asarray((camera.get('images') or {}).get('ego_view'))
    report['image_shape']=list(image.shape);report['image_dtype']=str(image.dtype)
    if image.ndim!=3 or image.shape[2]!=3 or image.dtype!=np.uint8:reasons.append('image:invalid_shape_or_dtype')
    report['ok']=not reasons
    return report
