def sort_rooms(rooms):
    res = []
    last = 0
    for group in [[1,6], [2,5], [3,4]]:
        subrooms = [r for r in rooms if r in group]
        subrooms.sort(reverse = last > 3.5)
        if len(subrooms) != 0:
            last = subrooms[-1]
            res.extend(subrooms)
    
    return res
