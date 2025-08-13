# SegmentationResult.msg
# Custom message for segmentation results

# Counter untuk tracking segmentation
int32 count

# Timestamp dalam detik (epoch time)
float64 timestamp

# Array deteksi objek untuk setiap derajat (360 elemen)
# Nilai 99 = tidak ada deteksi, 100 = placeholder, lainnya = hazard level
int32[] detected
