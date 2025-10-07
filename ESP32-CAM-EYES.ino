/**
 * @file main.cpp
 * @brief ESP32 Camera project to track a light source and display animated eyes on two round GC9A01A screens.
 *
 * This code initializes an ESP32 camera, processes the video feed to find the largest,
 * most circular bright blob, and then uses the blob's position to control the irises
 * of two animated eyes displayed on separate round TFT screens.
 */

// --- INCLUDES ---
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_GC9A01A.h"
#include "eye512_small_iris.h"
#include "esp_camera.h"

// --- CAMERA PIN DEFINITIONS (ESP32-CAM AI-Thinker Module) ---
// Note: To use a different module, ensure these pin definitions are correct.
// You can copy the correct pin assignment from the CameraWebServer example (camera_pins.h).
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1 // -1 means not used
#define XCLK_GPIO_NUM   0
#define SIOD_GPIO_NUM   26 // I2C Data
#define SIOC_GPIO_NUM   27 // I2C Clock
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM     5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22
#define LED_GPIO_NUM    4  // Onboard flash LED

// --- DISPLAY & SPI PIN DEFINITIONS ---
// Common SPI pins
#define SCK_PIN         14
#define MOSI_PIN        13
#define MISO_PIN        4  // Not typically used for TFTs, but defined for SPI
#define SS_PIN          2  // Generic SS, not used directly by TFTs

// Per-display specific pins
#define TFT_DC          12 // Data/Command pin for both displays
#define TFT_L_CS        15 // Chip Select for the LEFT display
#define TFT_R_CS        16 // Chip Select for the RIGHT display

// --- GLOBAL OBJECTS ---
// Create two instances of the Adafruit GC9A01A driver, one for each eye/display.
Adafruit_GC9A01A tft_L(TFT_L_CS, TFT_DC);
Adafruit_GC9A01A tft_R(TFT_R_CS, TFT_DC);

// --- DATA STRUCTURES ---
/**
 * @brief Structure to hold the final result of the light source detection.
 */
typedef struct {
  float x;          // Centroid X, normalized to the range [-1.0, 1.0]
  float y;          // Centroid Y, normalized to the range [-1.0, 1.0]
  int pixel_count;  // Total number of pixels in the detected blob
} light_source_result_t;

/**
 * @brief Structure to hold statistical information about each blob during image processing.
 *        Includes fields for second-order moments used for shape analysis (e.g., circularity).
 */
typedef struct {
  int id;           // Unique ID for the blob
  int pixel_count;  // Total pixels in the blob
  uint32_t sum_x;   // Sum of all X coordinates
  uint32_t sum_y;   // Sum of all Y coordinates
  uint32_t sum_x2;  // Sum of all X^2 values
  uint32_t sum_y2;  // Sum of all Y^2 values
  uint32_t sum_xy;  // Sum of all X*Y values
} blob_info_t;


/**
 * @brief Standard setup function, runs once at startup.
 */
void setup() {
  // Initialize serial communication for debugging.
  Serial.begin(9600);

  // Initialize the SPI bus with the defined pins.
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);

  // Initialize the left and right displays.
  // 80MHz is a common high-speed SPI frequency for these displays.
  tft_L.begin(80000000);
  tft_R.begin(80000000);

  // Set display rotations. The eyes might be mounted differently.
  // Rotation 1 is 90 degrees clockwise, 3 is 270 degrees.
  tft_L.setRotation(1);
  tft_R.setRotation(3);

  // --- CAMERA CONFIGURATION ---
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
  config.frame_size = FRAMESIZE_QQVGA; // 160x120 resolution, good for performance
  config.pixel_format = PIXFORMAT_RGB565; // Each pixel is 2 bytes
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_DRAM; // Use internal RAM. PSRAM not needed.
  config.jpeg_quality = 12; // Not used for raw RGB565 frames
  config.fb_count = 1; // Use one frame buffer

  // --- CAMERA INITIALIZATION ---
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // --- SENSOR SETTINGS ---
  // Access the camera sensor to fine-tune settings.
  sensor_t *s = esp_camera_sensor_get();
  s->set_exposure_ctrl(s, 0); // Disable automatic exposure control
  s->set_aec2(s, 0);          // Disable automatic exposure control (alternative)
  s->set_ae_level(s, -4);     // Set a fixed, low exposure level to better see bright lights
  s->set_lenc(s, 0);          // Disable lens correction
  s->set_aec_value(s, 0);     // Set manual exposure value to minimum
}


// --- DISJOINT SET UNION (DSU) / UNION-FIND ---
// This data structure efficiently manages equivalent blob labels during the first pass of labeling.
const int MAX_LABELS = 256; // Max number of blobs to track. Adjust if needed.
int parent[MAX_LABELS];

/**
 * @brief Initialize DSU: each label is its own parent initially.
 */
void dsu_init() {
  for (int i = 0; i < MAX_LABELS; i++) {
    parent[i] = i;
  }
}

/**
 * @brief Find the representative (root) of the set containing label i, with path compression.
 * @param i The label to find the root for.
 * @return The root label of the set.
 */
int dsu_find(int i) {
  if (parent[i] == i) {
    return i;
  }
  // Path compression: set parent directly to the root for faster future lookups.
  return parent[i] = dsu_find(parent[i]);
}

/**
 * @brief Unite the sets containing labels i and j.
 * @param i The first label.
 * @param j The second label.
 */
void dsu_unite(int i, int j) {
  int root_i = dsu_find(i);
  int root_j = dsu_find(j);
  if (root_i != root_j) {
    // Simple union: make the smaller-valued label the parent for consistency.
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
    Serial.println("Unsupported pixel format. Requires PIXFORMAT_RGB565.");
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

    // Color threshold for lightsource. Might need tweaking for colored lights.
    if ((r > 230) && (g > 230) && (b > 230)) {
        pixels[i] = 0xFFFF; // Mark as foreground
    } else {
        pixels[i] = 0;      // Mark as background
    }
  }

  // --- 2. FIRST PASS: LABELING AND RECORDING EQUIVALENCES ---
  dsu_init();
  int next_label = 1; // Start labeling from 1, since 0 is background.
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int i = y * width + x;
      if (pixels[i] == 0xFFFF) { // Found an unlabeled foreground pixel
        // Check neighbors (top and left) that have already been processed.
        uint16_t top_label = (y > 0) ? pixels[i - width] : 0;
        uint16_t left_label = (x > 0) ? pixels[i - 1] : 0;

        if (top_label == 0 && left_label == 0) { // New blob
          if (next_label < MAX_LABELS) {
            pixels[i] = next_label++;
          } else {
            pixels[i] = 0; // Out of labels, ignore this blob
          }
        } else if (top_label > 0 && left_label == 0) { // Part of blob from above
          pixels[i] = top_label;
        } else if (top_label == 0 && left_label > 0) { // Part of blob from the left
          pixels[i] = left_label;
        } else { // Touches two different labeled regions
          // This pixel connects two previously separate blobs.
          if (top_label != left_label) {
            dsu_unite(top_label, left_label); // Record that these labels are equivalent.
          }
          // Assign the smaller of the two labels.
          pixels[i] = (top_label < left_label) ? top_label : left_label;
        }
      }
    }
  }

  // --- 3. SECOND PASS: RESOLVE LABELS AND CALCULATE BLOB STATS ---
  blob_info_t blobs[MAX_LABELS] = {0};
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int i = y * width + x;
      if (pixels[i] > 0) { // If it's a foreground pixel
        // Find the final, root label for this pixel using the DSU.
        int final_label = dsu_find(pixels[i]);
        pixels[i] = final_label;

        // Aggregate statistics for this blob.
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

  // --- 4. FIND THE BEST BLOB (LARGEST AND MOST CIRCULAR) ---
  int best_blob_index = -1;
  int max_pixel_count = 0;

  const int min_blob_size = 3;         // Noise filter: ignore tiny blobs
  const float min_circularity = 0.5f;  // Shape filter: must be at least 50% circular

  for (int i = 1; i < next_label; i++) {
    // Only consider blobs that are roots of a set and meet the size criteria.
    if (blobs[i].id != 0 && blobs[i].pixel_count >= min_blob_size) {
      float N = (float)blobs[i].pixel_count;
      float cx = (float)blobs[i].sum_x / N;
      float cy = (float)blobs[i].sum_y / N;

      // Calculate second-order central moments (mu_xx, mu_yy, mu_xy).
      float mu_xx = (float)blobs[i].sum_x2 - cx * (float)blobs[i].sum_x;
      float mu_yy = (float)blobs[i].sum_y2 - cy * (float)blobs[i].sum_y;
      float mu_xy = (float)blobs[i].sum_xy - cx * (float)blobs[i].sum_y;

      // Calculate eigenvalues of the covariance matrix to find the major and minor axes of the fitted ellipse.
      float A = mu_xx + mu_yy;
      float B = sqrtf(powf(mu_xx - mu_yy, 2) + 4.0f * mu_xy * mu_xy);
      float lambda_max = (A + B) / 2.0f;
      float lambda_min = (A - B) / 2.0f;

      // Circularity is the ratio of the minor to major axis.
      // A perfect circle has a circularity of 1.0.
      float circularity = (lambda_max > 0.0f) ? sqrtf(lambda_min / lambda_max) : 0.0f;

      // Selection criteria: Must be circular enough, then pick the largest one.
      if (circularity >= min_circularity) {
        if (blobs[i].pixel_count > max_pixel_count) {
          max_pixel_count = blobs[i].pixel_count;
          best_blob_index = i;
        }
      }
    }
  }

  // If no suitable blob was found, return failure.
  if (best_blob_index == -1) {
    result->x = 0.0f;
    result->y = 0.0f;
    result->pixel_count = 0;
    return -1;
  }

  // --- 5. CALCULATE CENTROID AND FINALIZE RESULT ---
  blob_info_t* best_blob = &blobs[best_blob_index];
  result->pixel_count = best_blob->pixel_count;
  int res_x = best_blob->sum_x / best_blob->pixel_count;
  int res_y = best_blob->sum_y / best_blob->pixel_count;

  // Normalize coordinates from image space [0, width] to a centered space [-1.0, 1.0].
  result->x = (res_x / (float)width) * 2.0f - 1.0f;
  result->y = (res_y / (float)height) * 2.0f - 1.0f;

  // --- (Optional) VISUALIZATION FOR DEBUGGING ---
  // Overwrite the label data in the buffer with red crosshairs at the centroid.
  // Horizontal line
  /*
  for (int i = 0; i < width; i++) {
    pixels[res_y * width + i] = 0xF800; // Red in RGB565 format
  }
  // Vertical line
  for (int i = 0; i < height; i++) {
    pixels[i * width + res_x] = 0xF800; // Red in RGB565 format
  }
  */

  return 0; // Success
}

/**
 * @brief Main program loop.
 */
void loop() {
  camera_fb_t* fb = esp_camera_fb_get(); // Capture a frame from the camera.
  light_source_result_t result = {0};
  float aspect = 1.0f;

  if (!fb) {
    Serial.println("Camera capture failed");
  } else {
    // Process the frame if it's in the correct format.
    if (fb->format == PIXFORMAT_RGB565) {
      find_light_source(fb, &result);
      aspect = ((float)fb->width) / fb->height;
    } else {
      Serial.println("Wrong Format");
    }
    //Uncomment the following line and comment out the right drawEye call below in order
    //to draw the framebuffer to the right screen.
    //drawCameraImage(tft_R, fb);
    // Return the frame buffer to be reused.
    esp_camera_fb_return(fb);
    fb = NULL;
  }

  // --- CALCULATE EYE OFFSETS ---
  // Invert the X coordinate because the camera's view is a mirror of what the eyes should "see".
  float fx = -result.x;
  float fy = result.y;

  // Animate the "pupil" size based on the detected blob size (not used in drawing).
  int size = (int)(result.pixel_count * 0.05f);

  // Calculate the final X and Y offsets for the eye texture.
  // '120' is the center of the 240x240 display.
  int y_ofst = (int)(120.0f + fy * 120);

  // Apply some exaggeration for horizontal movement to create a slightly more dynamic effect.
  // If the light is to the right (fx > 0), the left eye looks further right.
  // If the light is to the left (fx < 0), the right eye looks further left.
  int x_ofst_l = (int)(120.0f + (fx > 0 ? fx : 1.25 * fx) * 100 * aspect + size);
  int x_ofst_r = (int)(120.0f + (fx < 0 ? fx : 1.25 * fx) * 100 * aspect - size);

  // --- DRAW THE EYES ---
  drawEye(tft_L, x_ofst_l, y_ofst);
  drawEye(tft_R, x_ofst_r, y_ofst);
}

/**
 * @brief Draws a 240x240 portion of the large eye texture onto a display.
 * @param tft The display object to draw on.
 * @param x The desired horizontal center of the iris in the final image.
 * @param y The desired vertical center of the iris in the final image.
 */
void drawEye(Adafruit_GC9A01A tft, int x, int y) {
  tft.startWrite(); // Begin SPI transaction
  tft.setAddrWindow(0, 0, 240, 240); // Set the drawing area to the full screen

  // Calculate the top-left corner (x_eye_ofst, y_eye_ofst) of the 240x240 window
  // within the larger eye texture bitmap.
  int x_eye_ofst = eye_width / 2 - x;
  // Constrain the offset to prevent drawing outside the texture boundaries.
  // '30' provides a margin.
  x_eye_ofst = max(30, min(x_eye_ofst, eye_width - 240 - 30));

  int y_eye_ofst = eye_height / 2 - y;
  y_eye_ofst = max(30, min(y_eye_ofst, eye_height - 240 - 30));

  // Stride is the width of the source texture in pixels.
  int eye_stride = eye_width;

  // Calculate the starting pointer in the eye_data array.
  uint16_t* eye_ptr = (uint16_t*)(eye_data + y_eye_ofst * eye_stride + x_eye_ofst);

  // Draw the 240x240 window row by row.
  for (int i = 0; i < 240; i++) {
    tft.writePixels(eye_ptr, 240); // Send one row of 240 pixels
    eye_ptr += eye_stride;         // Move the pointer to the start of the next row in the source texture
  }

  tft.endWrite(); // End SPI transaction
}

/**
 * @brief Debug function to draw the raw camera image directly to a display.
 * @param tft The display to draw on.
 * @param fb The camera frame buffer to display.
 */
void drawCameraImage(Adafruit_GC9A01A tft, camera_fb_t* fb) {
  // The bytes in the RGB565 buffer might be in the wrong order for the display driver.
  // This loop swaps the bytes for each pixel (endianness correction).
  for (int i = 0; i < fb->width * fb->height; i++) {
    uint8_t tmp = fb->buf[i * 2];
    fb->buf[i * 2] = fb->buf[i * 2 + 1];
    fb->buf[i * 2 + 1] = tmp;
  }

  tft.startWrite();

  // Center the 160x120 camera image on the 240x240 display.
  int x_start = 120 - fb->width / 2;
  int y_start = 120 - fb->height / 2;
  tft.setAddrWindow(x_start, y_start, fb->width, fb->height);

  // Write the entire frame buffer to the screen at once.
  tft.writePixels((uint16_t*)fb->buf, fb->width * fb->height);

  tft.endWrite();
}