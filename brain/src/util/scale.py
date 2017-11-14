def scale(value, source_min, source_max, dest_min, dest_max):
    source_size = source_max - source_min
    dest_size = dest_max - dest_min
    if source_size == 0 or dest_size == 0:
        return dest_min

    scale_factor = source_size / float(dest_size)
    return (value - source_min) / scale_factor + dest_min
