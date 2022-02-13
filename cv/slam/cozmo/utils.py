def add_zeros(num_string, digits):
    num_string = str(num_string)
    while len(num_string) < digits:
        num_string = '0' + num_string
    return num_string