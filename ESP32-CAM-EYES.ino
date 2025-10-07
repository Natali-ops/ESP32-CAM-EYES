#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_GC9A01A.h"
#include "eye512_small_iris.h"
#include "esp_camera.h"

// CAMERA PINS:
// Note: To use a different module make sure to properly define the camera Pins.
// You can copy the correct pin assignment from the CameraWebServer example (camera_pins.h)
#define PWDN_GPIO_NUM  32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  0
#define SIOD_GPIO_NUM  26
#define SIOC_GPIO_NUM  27
#define Y9_GPIO_NUM    35
#define Y8_GPIO_NUM    34
#define Y7_GPIO_NUM    39
#define Y6_GPIO_NUM    36
#define Y5_GPIO_NUM    21
#define Y4_GPIO_NUM    19
#define Y3_GPIO_NUM    18
#define Y2_GPIO_NUM    5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM  23
#define PCLK_GPIO_NUM  22
#define LED_GPIO_NUM   4

//DISPLAY PINS:
#define TFT_DC         12
#define TFT_L_CS       15
#define TFT_R_CS       16

#define MISO_PIN       4
#define SS_PIN         2
#define MOSI_PIN       13
#define SCK_PIN        14

Adafruit_GC9A01A tft_L(TFT_L_CS, TFT_DC);
Adafruit_GC9A01A tft_R(TFT_R_CS, TFT_DC);

// Structure to hold the final result
typedef struct {
    float x; // Centroid X, normalized to [-1.0, 1.0]
    float y; // Centroid Y, normalized to [-1.0, 1.0]
    int pixel_count;
} light_source_result_t;

// Structure to hold information about each blob during processing.
// Includes fields for second-order moments for shape analysis.
typedef struct {
    int id;
    int pixel_count;
    uint32_t sum_x;
    uint32_t sum_y;
    uint32_t sum_x2; // Sum of x^2
    uint32_t sum_y2; // Sum of y^2
    uint32_t sum_xy; // Sum of x*y
} blob_info_t;

void setup() {
  Serial.begin(9600);

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);

  tft_L.begin(80000000);
  tft_R.begin(80000000);
  tft_L.setRotation(1);
  tft_R.setRotation(3);

  //Camera:
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 40000000;
  config.frame_size = FRAMESIZE_QQVGA;
  config.pixel_format = PIXFORMAT_RGB565;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  //config.fb_location = CAMERA_FB_IN_PSRAM;
  config.fb_location = CAMERA_FB_IN_DRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_exposure_ctrl(s, 0);
  s->set_aec2(s,0);
  s->set_ae_level(s, -4);
  s->set_lenc(s, 0);
  s->set_aec_value(s, 0);
}

// --- Disjoint Set Union (DSU) / Union-Find ---
// This data structure efficiently manages equivalent blob labels.
const int MAX_LABELS = 256; // Max number of blobs to track. Adjust if needed.
int parent[MAX_LABELS];

// Initialize DSU: each label is its own parent initially.
void dsu_init() {
    for (int i = 0; i < MAX_LABELS; i++) {
        parent[i] = i;
    }
}

// Find the representative (root) of the set containing label i, with path compression.
int dsu_find(int i) {
    if (parent[i] == i) {
        return i;
    }
    // Path compression: set parent directly to the root for faster future lookups.
    return parent[i] = dsu_find(parent[i]);
}

// Unite the sets containing labels i and j.
void dsu_unite(int i, int j) {
    int root_i = dsu_find(i);
    int root_j = dsu_find(j);
    if (root_i != root_j) {
        // Simple union: make the smaller-valued label the parent.
        if (root_i < root_j) {
            parent[root_j] = root_i;
        } else {
            parent[root_i] = root_j;
        }
    }
}


/**
 * @brief Finds the largest, most circular bright blob in a camera frame.
 *
 * This function implements a two-pass connected component labeling algorithm
 * combined with moment analysis to identify a light source.
 *
 * @param fb Pointer to the camera frame buffer (must be PIXFORMAT_RGB565).
 * @param result Pointer to a struct where the result will be stored.
 * @return 0 on success, -1 on failure (e.g., wrong format, no suitable blob found).
 */
int find_light_source(camera_fb_t* fb, light_source_result_t* result) {
    if (fb->format != PIXFORMAT_RGB565) {
        log_e("Unsupported pixel format. Requires PIXFORMAT_RGB565.");
        return -1;
    }

    uint16_t* pixels = (uint16_t*)fb->buf;
    const int width = fb->width;
    const int height = fb->height;

    // --- 1. Binarization ---
    // Convert the image to a binary format (0 for background, 0xFFFF for foreground)
    // based on a color threshold. This is done in-place to save memory.
    for (int i = 0; i < width * height; i++) {

        uint8_t low_byte = *((uint8_t*)(pixels+i));
        uint8_t high_byte = *(((uint8_t*)(pixels+i))+1);

        uint8_t r = low_byte & 0xF8; 
        uint8_t g = (low_byte & 0x07) << 5 | (high_byte & 0xE0) >> 3;
        uint8_t b = (high_byte & 0x1F) << 3;

        /*
        r = g;
        b = g;
        low_byte = (r & 0xF8) | ((g >> 5) & 0x07);
        high_byte = ((g << 3) & 0xE0) | ((b >> 3) & 0x1F);
        
        *((uint8_t*)(pixels+i)) = low_byte;
        *(((uint8_t*)(pixels+i))+1) = high_byte;
        */

        // Threshold for a bright, slightly greenish-blue light
        if ((g > 230) && (r > 230) && (b > 230)) {
            pixels[i] = 0xFFFF; // Mark as foreground
        } else {
            pixels[i] = 0;      // Mark as background
        }
        
    }


    // --- 2. First Pass: Labeling and Recording Equivalences ---
    dsu_init();
    int next_label = 1;
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int i = y * width + x;
            if (pixels[i] == 0xFFFF) { // Found a foreground pixel
                // Check top and left neighbors
                uint16_t top_label = (y > 0) ? pixels[i - width] : 0;
                uint16_t left_label = (x > 0) ? pixels[i - 1] : 0;

                if (top_label == 0 && left_label == 0) { // New blob
                    if (next_label < MAX_LABELS) {
                        pixels[i] = next_label++;
                    } else {
                        pixels[i] = 0; // Out of labels, ignore this blob
                    }
                } else if (top_label > 0 && left_label == 0) { // Part of top blob
                    pixels[i] = top_label;
                } else if (top_label == 0 && left_label > 0) { // Part of left blob
                    pixels[i] = left_label;
                } else { // Touches two blobs
                    if (top_label != left_label) {
                        dsu_unite(top_label, left_label); // Record equivalence
                    }
                    pixels[i] = (top_label < left_label) ? top_label : left_label;
                }
            }
        }
    }

    // --- 3. Second Pass: Resolve Labels and Calculate Blob Stats ---
    blob_info_t blobs[MAX_LABELS] = {0};
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            int i = y * width + x;
            if (pixels[i] > 0) {
                int final_label = dsu_find(pixels[i]);
                pixels[i] = final_label;

                if (blobs[final_label].id == 0) blobs[final_label].id = final_label;

                blobs[final_label].pixel_count++;
                blobs[final_label].sum_x += x;
                blobs[final_label].sum_y += y;
                blobs[final_label].sum_x2 += (uint32_t)x * x;
                blobs[final_label].sum_y2 += (uint32_t)y * y;
                blobs[final_label].sum_xy += (uint32_t)x * y;
            }
        }
    }

    // --- 4. Find the Best Blob (Largest and Most Circular) ---
    int best_blob_index = -1;
    int max_pixel_count = 0;

    const int min_blob_size = 3; // Noise filter: ignore tiny blobs
    const float min_circularity = 0.5f; // Shape filter: must be at least 50% circular

    for (int i = 1; i < next_label; i++) {
        if (blobs[i].id != 0 && blobs[i].pixel_count >= min_blob_size) {
            float N = (float)blobs[i].pixel_count;
            float cx = (float)blobs[i].sum_x / N;
            float cy = (float)blobs[i].sum_y / N;

            // Calculate second-order central moments
            float mu_xx = (float)blobs[i].sum_x2 - cx * (float)blobs[i].sum_x;
            float mu_yy = (float)blobs[i].sum_y2 - cy * (float)blobs[i].sum_y;
            float mu_xy = (float)blobs[i].sum_xy - cx * (float)blobs[i].sum_y;

            // Calculate eigenvalues of the covariance matrix to find ellipse axes
            float A = mu_xx + mu_yy;
            float B = sqrtf(powf(mu_xx - mu_yy, 2) + 4.0f * mu_xy * mu_xy);
            float lambda_max = (A + B) / 2.0f;
            float lambda_min = (A - B) / 2.0f;

            // Circularity is the ratio of the minor to major axis of the fitted ellipse.
            // A perfect circle has a circularity of 1.0.
            float circularity = (lambda_max > 0.0f) ? sqrtf(lambda_min / lambda_max) : 0.0f;

            // Selection criteria: Must be circular enough, then pick the largest.
            if (circularity >= min_circularity) {
                if (blobs[i].pixel_count > max_pixel_count) {
                    max_pixel_count = blobs[i].pixel_count;
                    best_blob_index = i;
                }
            }
        }
    }

    if (best_blob_index == -1) {
        // No suitable blobs found that met all criteria
        result->x = 0.0f;
        result->y = 0.0f;
        result->pixel_count = 0;
        return -1;
    }

    // --- 5. Calculate Centroid and Finalize Result ---
    blob_info_t* best_blob = &blobs[best_blob_index];
    result->pixel_count = best_blob->pixel_count;
    int res_x = best_blob->sum_x / best_blob->pixel_count;
    int res_y = best_blob->sum_y / best_blob->pixel_count;

    // Normalize coordinates from image space to [-1.0, 1.0]
    result->x = (res_x / (float)width) * 2.0f - 1.0f;
    result->y = (res_y / (float)height) * 2.0f - 1.0f;

    // --- (Optional) Visualization: Draw crosshairs on the buffer for debugging ---
    // Note: This overwrites the label data in the buffer with red pixels.
    // Horizontal line
    /*
    for (int i = 0; i < width; i++) {
        pixels[res_y * width + i] = 0xF800; // Red in RGB565
    }
    // Vertical line
    for (int i = 0; i < height; i++) {
        pixels[i * width + res_x] = 0xF800; // Red in RGB565
    }
    */

    return 0; // Success
}

void loop() {
  camera_fb_t* fb = esp_camera_fb_get();
  light_source_result_t result = {0};
  float aspect = 1.0f;
  if (!fb) {
    log_e("Camera capture failed");
  } else {
    if (fb->format != PIXFORMAT_RGB565) {
      log_e("Wrong Format");
    } else {
      find_light_source(fb, &result);
      aspect = ((float)fb->width) / fb->height;
      //Uncomment the following line and comment out the right drawEye call below in order
      //to draw the framebuffer to the right screen.
      //drawCameraImage(tft_R, fb);
      esp_camera_fb_return(fb);
      fb = NULL;
    }
  }

  float fx = -result.x;
  float fy = result.y;
  int size = (int)(result.pixel_count * 0.05f);
  int y_ofst = (int)(120.0f + fy * 100);
  int x_ofst_l = (int)(120.0f + (fx>0?fx:1.25*fx) * 100 * aspect + size);
  int x_ofst_r = (int)(120.0f + (fx<0?fx:1.25*fx) * 100 * aspect - size);
  drawEye(tft_L, x_ofst_l, y_ofst);
  drawEye(tft_R, x_ofst_r, y_ofst);
}

void drawEye(Adafruit_GC9A01A tft, int x, int y) {
  tft.startWrite();
  tft.setAddrWindow(0, 0, 240, 240); // Clipped area

  int x_eye_ofst = eye_width/2 - x;
  x_eye_ofst = max(30, min(x_eye_ofst, eye_width-240-30));
  int y_eye_ofst = eye_height/2 - y;
  y_eye_ofst = max(30, min(y_eye_ofst, eye_height-240-30));
  int eye_stride = eye_width;
  uint16_t* eye_ptr = (uint16_t*)(eye_data + y_eye_ofst * eye_stride + x_eye_ofst);
  for(int i=0; i<240; i++) {
    tft.writePixels(eye_ptr, 240);
    eye_ptr += eye_stride;
  }
  tft.endWrite();
}

void drawCameraImage(Adafruit_GC9A01A tft, camera_fb_t* fb) {
  for(int i=0; i<fb->width*fb->height; i++) {
    uint8_t tmp = fb->buf[i*2];
    fb->buf[i*2] = fb->buf[i*2 + 1];
    fb->buf[i*2 + 1] = tmp;
  }
  tft.startWrite();
  tft.setAddrWindow(0, 0, 240, 240); // Clipped area
  int x_start = 120 - fb->width/2;
  int y_start = 120 - fb->height/2;
  tft.setAddrWindow(x_start, y_start, fb->width, fb->height);
  tft.writePixels((uint16_t*)fb->buf, fb->width * fb->height);
  tft.endWrite();
}
