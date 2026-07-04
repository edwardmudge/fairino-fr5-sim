import polyscope as ps
from geometry_backend import VisContent
from gui_panel import UI_Menu

def main():
    # 1. Initialise the Polyscope system
    ps.init()
    ps.set_program_name("Geometry Processing Lab")
    ps.set_ground_plane_mode("none") # Clean mode
    ps.set_up_dir("z_up")

    # 2. Instantiate the backend (Model)
    backend = VisContent()

    # 3. Instantiate the frontend (View), and inject the backend into it
    ui = UI_Menu(backend)

    # 4. Define the main loop callback
    # Polyscope is a C++ binding and requires a parameter-less function as the callback
    def callback_loop():
        ui.render()

    # 5. Register the callback and start
    ps.set_user_callback(callback_loop)
    ps.show()

if __name__ == "__main__":
    main()
