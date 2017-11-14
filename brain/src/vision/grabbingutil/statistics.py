from numpy import NaN, Inf, arange, isscalar, asarray

def find_peaks(v, delta, x = None):
    """
    Converted from MATLAB script at http://billauer.co.il/peakdet.html
    Currently returns two lists of tuples, but maybe arrays would be better
    function [maxtab, mintab]=peakdet(v, delta, x)
    
    PEAKDET Detect peaks in a vector
     [MAXTAB, MINTAB] = PEAKDET(V, DELTA) finds the local
     maxima and minima ("peaks") in the vector V.
     MAXTAB and MINTAB consists of two columns. Column 1
     contains indices in V, and column 2 the found values.
    
     With [MAXTAB, MINTAB] = PEAKDET(V, DELTA, X) the indices
     in MAXTAB and MINTAB are replaced with the corresponding
     X-values.
    
     A point is considered a maximum peak if it has the maximal
     value, and was preceded (to the left) by a value lower by
     DELTA.
     Eli Billauer, 3.4.05 (Explicitly not copyrighted).
     This function is released to the public domain; Any use is allowed.
    """
    
    maxtab = []
    mintab = []
       
    if x is None:
        x = arange(len(v))
    
    v = asarray(v)
    
    if len(v) != len(x):
        sys.exit('Input vectors v and x must have same length')
    
    if not isscalar(delta):
        sys.exit('Input argument delta must be a scalar')
    
    if delta <= 0:
        sys.exit('Input argument delta must be positive')
    
    mn, mx = Inf, -Inf
    mnpos, mxpos = NaN, NaN
    
    lookformax = True
    
    for i in arange(len(v)):
        this = v[i]
        if this > mx:
            mx = this
            mxpos = x[i]
        if this < mn:
            mn = this
            mnpos = x[i]
        
        if lookformax:
            if this < mx-delta:
                maxtab.append(mxpos)
                mn = this
                mnpos = x[i]
                lookformax = False
        else:
            if this > mn+delta:
                mintab.append(mnpos)
                mx = this
                mxpos = x[i]
                lookformax = True

    return maxtab, mintab

def find_foot(data, peak_idx, threshold, direction):
    idx = peak_idx
    total_sum = data[idx]
    
    while idx < len(data) - 1 if direction > 0 else idx > 0:
        new_total_sum = total_sum + data[idx + direction]
        diff = new_total_sum - total_sum
        if diff / new_total_sum < threshold:
            break
        total_sum = new_total_sum
        idx = idx + direction
    
    return idx

def find_hill(data, peak_idx, threshold):
    return (
        find_foot(data, peak_idx, threshold, -1),
        find_foot(data, peak_idx, threshold,  1)
    )
