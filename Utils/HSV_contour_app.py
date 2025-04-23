import tkinter as tk
from tkinter import filedialog
import cv2
import numpy as np
from PIL import Image, ImageTk

class HSVContourApp:
    def __init__(self, root):
        self.root = root
        self.root.title("HSV Contour Extractor with Mask and Contours")
        self.image = None
        self.processed_image = None

        self.frame = tk.Frame(self.root)
        self.frame.pack(padx=10, pady=10)

        self.upload_btn = tk.Button(self.frame, text="Upload Image", command=self.upload_image)
        self.upload_btn.grid(row=0, column=0, columnspan=2, pady=5)

        self.sliders = {}
        self.slider_labels = {}
        labels = [
            ("Hue Min", 0, 179, 0),
            ("Hue Max", 0, 179, 179),
            ("Sat Min", 0, 255, 0),
            ("Sat Max", 0, 255, 255),
            ("Val Min", 0, 255, 0),
            ("Val Max", 0, 255, 255),
        ]
        for i, (name, min_val, max_val, default) in enumerate(labels):
            tk.Label(self.frame, text=name).grid(row=i+1, column=0, sticky="w")
            self.sliders[name] = tk.Scale(self.frame, from_=min_val, to=max_val, orient=tk.HORIZONTAL,
                                          length=200, command=self.update_image)
            self.sliders[name].set(default)
            self.sliders[name].grid(row=i+1, column=1, pady=2)

        self.original_canvas = tk.Canvas(self.frame, width=400, height=400)
        self.original_canvas.grid(row=1, column=2, rowspan=7, padx=10)
        self.processed_canvas = tk.Canvas(self.frame, width=400, height=400)
        self.processed_canvas.grid(row=1, column=3, rowspan=7, padx=10)

        tk.Label(self.frame, text="Original Image").grid(row=0, column=2)
        tk.Label(self.frame, text="Masked Image with Contours").grid(row=0, column=3)

    def upload_image(self):
        file_path = filedialog.askopenfilename(filetypes=[("Image files", "*.jpg *.jpeg *.png")])
        if file_path:
            self.image = cv2.imread(file_path)
            self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
            self.update_image()

    def update_image(self, *args):
        if self.image is None:
            return

        hue_min = self.sliders["Hue Min"].get()
        hue_max = self.sliders["Hue Max"].get()
        sat_min = self.sliders["Sat Min"].get()
        sat_max = self.sliders["Sat Max"].get()
        val_min = self.sliders["Val Min"].get()
        val_max = self.sliders["Val Max"].get()

        hsv = cv2.cvtColor(self.image, cv2.COLOR_RGB2HSV)
        lower_bound = np.array([hue_min, sat_min, val_min])
        upper_bound = np.array([hue_max, sat_max, val_max])
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        mask_filled = np.zeros_like(mask)
        for contour in contours:
            cv2.fillPoly(mask_filled, [contour], 255)

        masked_image = cv2.bitwise_and(self.image, self.image, mask=mask_filled)

        cv2.drawContours(masked_image, contours, -1, (255, 0, 0), 2)

        self.display_image(self.image, self.original_canvas)
        self.display_image(masked_image, self.processed_canvas)

    def display_image(self, img, canvas):
        max_size = 400
        h, w = img.shape[:2]
        scale = min(max_size / w, max_size / h)
        new_size = (int(w * scale), int(h * scale))
        resized = cv2.resize(img, new_size, interpolation=cv2.INTER_AREA)

        img_pil = Image.fromarray(resized)
        img_tk = ImageTk.PhotoImage(img_pil)
        canvas.create_image(max_size // 2, max_size // 2, image=img_tk, anchor="center")
        canvas.image = img_tk

if __name__ == "__main__":
    root = tk.Tk()
    app = HSVContourApp(root)
    root.mainloop()