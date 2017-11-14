def get_object_center(dictionary, sizex, sizey):
    """This returns the direction of an observed object. If part of the object is outside the screen, false_pos is returned instead."""
    coor = dictionary['corners']
    x1, x2, x3, x4 = coor[0][0], coor[1][0], coor[2][0], coor[3][0]       
    y1, y2, y3, y4 = coor[0][1], coor[1][1], coor[2][1], coor[3][1]

    # Object touches contours of image
    if (x1 > sizex or x1 < 0 or x2 > sizex or x2 < 0 or x3 > sizex or x3 < 0 or x4 > sizex or x4 < 0 or y1 > sizey or y2 > sizey or y3 < 0 or y4 < 0):
        return False
    
    # Object too small/large    
    if abs(x1 - x2) < 30 or abs(x1-x2) > 400:
        return False

    # Object center in pixel coordinates
    x_pos = (x1 + x2 + x3 + x4)/4 - (sizex/2) 
    y_pos = (y1 + y2 + y3 + y4)/4 - (sizey/2)
            
    return x_pos, y_pos