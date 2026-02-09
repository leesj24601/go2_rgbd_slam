try:
    from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
    print(f"ISAACLAB_NUCLEUS_DIR: {ISAACLAB_NUCLEUS_DIR}")
except ImportError:
    print("Could not import isaaclab")
except Exception as e:
    print(f"Error: {e}")
