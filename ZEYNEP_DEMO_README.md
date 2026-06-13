# Zeynep Mission Control (MATLAB) Demo Başlatıcı

Bu klasör, tezin MATLAB Arayüzü ile ROS 2 Nav2 otonom altyapısının birleşimini test etmek için özel olarak hazırlanmıştır.

## 🚀 Sistemi Nasıl Başlatırım?

Aşağıdaki komutu çalıştırdığınızda; arka planda eski süreçler temizlenir, Gazebo otonom simülasyonu başlatılır ve sistem hazır olduğunda otomatik olarak **Zeynep Mission Control (MATLAB)** arayüzü açılır.

```bash
./RUN_ZEYNEP_MATLAB_DEMO.sh
```

**Not:** Bu komut varsayılan olarak `leo_obstacles.sdf` haritasını açar. (Bu harita tünel ve engel senaryoları için tasarlanmıştır).

---

## 🌍 Farklı Haritaları (Dünyaları) Nasıl Açarım?

Eğer sunum esnasında veya testlerde farklı bir Gazebo dünyası açmak isterseniz, `RUN_ZEYNEP_MATLAB_DEMO.sh` dosyasının içindeki başlatma komutuna `--world` parametresini ekleyebilirsiniz.

Örnekler:
* **Endüstriyel Harita:**
  Dosyanın içindeki `run_hybrid_test.sh` satırını şu şekilde değiştirin:
  `./run_hybrid_test.sh --world industrial --no-mission`

* **Clearpath Haritası:**
  `./run_hybrid_test.sh --world clearpath --no-mission`

* **Boş Harita:**
  `./run_hybrid_test.sh --world empty --no-mission`

## ⚙️ Arka Plan Bilgisi (Necati İçin)
Bu başlatıcı (`--no-mission`), Nav2, UKF, SLAM ve GPS Spoofer altyapısının **tamamını kullanır**. Sadece `mission_manager.py` (statik YAML okuyucu) scriptini devre dışı bırakıp, hedef belirleme ve Pre-Collision (Çarpışma Öncesi) güvenlik algoritmalarını tamamen dinamik MATLAB arayüzüne devreder.
