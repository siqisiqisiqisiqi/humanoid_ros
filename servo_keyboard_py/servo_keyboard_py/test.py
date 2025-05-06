from pynput import keyboard

def on_press(key):
    try:
        print(f"Key '{key.char}' pressed")
    except AttributeError:
        # Special keys (e.g. ctrl, alt, arrows) don’t have .char
        print(f"Special key {key} pressed")

def on_release(key):
    print(f"Key {key} released")
    # Stop listener on Esc
    if key == keyboard.Key.esc:
        print("Exiting on ESC")
        return False

if __name__ == "__main__":
    # Create and start the listener in the main thread
    with keyboard.Listener(
            on_press=on_press,
            on_release=on_release) as listener:
        print("Listening for keyboard events (press ESC to exit)...")
        listener.join()

