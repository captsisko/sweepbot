RANGE = 6.0
MAX_RANGE = 8.0

STATES = {
    # 'SEEK' : 0,
    'LOCATE-RIGHT-WALL' : 1,
    'FOLLOWKERB' : 3,
    'TURNLEFT' : 2,
    # 'DRIVE' : 4,
}

def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

# starboard_active_section = 'starboard_abeam_bow'
starboard_active_section = 'starboard_abeam'