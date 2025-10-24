import tkinter as tk

root = tk.Tk()
root.title("Frankenmolder")

label = tk.Label(root, text="Frankenmolder is running", font=("Helvetica", 24))
label.pack(padx=40, pady=40)

root.mainloop()