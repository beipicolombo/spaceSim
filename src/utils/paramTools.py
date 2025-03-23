


# --------------------------------------------------
# CLASSES
# --------------------------------------------------



# --------------------------------------------------
# FUNCTIONS
# --------------------------------------------------
def patchAttributes(obj, dicToPatch):
    objNew = obj
    for key, value in dicToPatch.items():
        setattr(obj, key, value)
        print("patching " + key + "...")
    return objNew