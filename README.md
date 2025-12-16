# 5 Serbestlik Dereceli (5-DOF) Robot Kol Simülasyonu

> **Language / Dil:** [🇹🇷 Türkçe](#5-serbestlik-dereceli-5-dof-robot-kol-simülasyonu) | [🇬🇧 English](#english-version)

## İçindekiler
1. [Proje Genel Bakış](#1-proje-genel-bakış)
2. [Kinematik Yapı ve Eklem Konfigürasyonu](#2-kinematik-yapı-ve-eklem-konfigürasyonu)
3. [İleri Kinematik (Forward Kinematics)](#3-i̇leri-kinematik-forward-kinematics)
4. [Ters Kinematik (Inverse Kinematics)](#4-ters-kinematik-inverse-kinematics)
5. [Jacobian Matrisi ve Hız Kinematiği](#5-jacobian-matrisi-ve-hız-kinematiği)
6. [Trajektori Planlama](#6-trajektori-planlama)
7. [3D Görselleştirme](#7-3d-görselleştirme)
8. [Kullanım Kılavuzu](#8-kullanım-kılavuzu)
9. [Teknik Detaylar](#9-teknik-detaylar)

---

## 1. Proje Genel Bakış

Bu proje, 5 serbestlik dereceli (5-DOF) bir robot kolun gerçek zamanlı simülasyonunu OpenGL ve C# kullanarak gerçekleştirmektedir. Proje, robotik alanındaki temel kinematik kavramları içermektedir:

- **İleri Kinematik (FK)**: Eklem açılarından uç efektör pozisyonunu hesaplama
- **Ters Kinematik (IK)**: Hedef pozisyondan eklem açılarını hesaplama
- **Jacobian Matrisi**: Eklem hızları ile uç efektör hızı arasındaki ilişki
- **Trajektori Planlama**: Yumuşak hareket geçişleri
- **3D Görselleştirme**: Gerçek zamanlı robot simülasyonu

### Teknoloji Yığını
- **Platform**: .NET Framework 4.7.2
- **Grafik Kütüphanesi**: OpenTK 3.3.3 (OpenGL)
- **Dil**: C# 
- **IDE**: Visual Studio / Visual Studio Code

---

## 2. Kinematik Yapı ve Eklem Konfigürasyonu

### 2.1 Eklem Tanımları

Robot kol 5 döner (revolute) eklemden oluşmaktadır:

| Eklem | İsim | Dönüş Ekseni | Açı Değişkeni | Hareket Sınırları | Açıklama |
|-------|------|--------------|---------------|-------------------|----------|
| J1 | Taban (Base) | Y ekseni | θ₁ | Sınırsız | Taban rotasyonu (Yaw) |
| J2 | Omuz (Shoulder) | X ekseni | θ₂ | [-90°, +90°] | Öne/arkaya eğilme (Pitch) |
| J3 | Dirsek (Elbow) | X ekseni | θ₃ | [-150°, +150°] | Kol bükülmesi (Pitch) |
| J4 | Bilek (Wrist Pitch) | X ekseni | θ₄ | [-90°, +90°] | Bilek yukarı/aşağı (Pitch) |
| J5 | Bilek Roll (Wrist Roll) | Y ekseni | θ₅ | Sınırsız | Uç efektör dönüşü (Yaw) |

### 2.2 Link Uzunlukları

Robot kolun fiziksel parametreleri:

```
L₁ = 1.0 birim  (Taban yüksekliği)
L₂ = 0.8 birim  (Omuz-Dirsek arası)
L₃ = 0.6 birim  (Dirsek-Bilek arası)
L₄ = 0.5 birim  (Bilek-Uç arası)
```

**Toplam Maksimum Erişim**: L₁ + L₂ + L₃ + L₄ = 2.9 birim

### 2.3 Koordinat Sistemi

Proje standart robotik koordinat sistemini kullanır:
- **X ekseni**: Kırmızı - Yatay (sağ/sol)
- **Y ekseni**: Yeşil - Dikey (yukarı/aşağı)
- **Z ekseni**: Mavi - Derinlik (ileri/geri)

Sağ el kuralı (right-hand rule) uygulanır.

---

## 3. İleri Kinematik (Forward Kinematics)

### 3.1 Matematiksel Formülasyon

İleri kinematik, eklem açıları vektörü **q** = [θ₁, θ₂, θ₃, θ₄, θ₅]ᵀ verildiğinde, uç efektörün (end-effector) pozisyonunu **p** = [x, y, z]ᵀ hesaplar.

#### Denavit-Hartenberg (DH) Yaklaşımı Yerine Doğrudan Matris Çarpımı

Bu implementasyon, DH parametreleri yerine doğrudan homojen transformasyon matrislerini kullanır:

```
T₀⁵ = T₀¹ · T¹² · T²³ · T³⁴ · T⁴⁵
```

Her eklem için transformasyon:

**Eklem 1 (Taban - Y ekseni dönüşü):**
```
T₀¹ = Rot(Y, θ₁) · Trans(0, L₁, 0)
```

**Eklem 2 (Omuz - X ekseni dönüşü):**
```
T¹² = Rot(X, θ₂) · Trans(0, L₂, 0)
```

**Eklem 3 (Dirsek - X ekseni dönüşü):**
```
T²³ = Rot(X, θ₃) · Trans(0, L₃, 0)
```

**Eklem 4 (Bilek Pitch - X ekseni dönüşü):**
```
T³⁴ = Rot(X, θ₄) · Trans(0, L₄, 0)
```

**Eklem 5 (Bilek Roll - Y ekseni dönüşü):**
```
T⁴⁵ = Rot(Y, θ₅)
```

### 3.2 Homojen Transformasyon Matrisleri

**Y ekseni etrafında dönüş:**
```
       ┌ cos(θ)   0   sin(θ)   0 ┐
Rʸ(θ) =│   0      1     0      0 │
       │-sin(θ)   0   cos(θ)   0 │
       └   0      0     0      1 ┘
```

**X ekseni etrafında dönüş:**
```
       ┌  1      0       0      0 ┐
Rˣ(θ) =│  0   cos(θ) -sin(θ)   0 │
       │  0   sin(θ)  cos(θ)   0 │
       └  0      0       0      1 ┘
```

**Translasyon matrisi:**
```
          ┌ 1  0  0  tₓ ┐
Trans(t) =│ 0  1  0  tᵧ │
          │ 0  0  1  tᵤ │
          └ 0  0  0  1  ┘
```

### 3.3 Kod İmplementasyonu

```csharp
Vector3 ComputeForwardKinematics(float θ₁, float θ₂, float θ₃, float θ₄, float θ₅)
{
    // Derece -> Radyan dönüşümü
    float t1 = DegreesToRadians(θ₁);
    float t2 = DegreesToRadians(θ₂);
    float t3 = DegreesToRadians(θ₃);
    float t4 = DegreesToRadians(θ₄);
    float t5 = DegreesToRadians(θ₅);

    Matrix4 T = Identity;
    
    // Eklem transformasyonlarını sırayla uygula
    T *= RotationY(t1) * Translation(0, L₁, 0);
    T *= RotationX(t2) * Translation(0, L₂, 0);
    T *= RotationX(t3) * Translation(0, L₃, 0);
    T *= RotationX(t4) * Translation(0, L₄, 0);
    T *= RotationY(t5);
    
    // Orijin noktasını transformasyon matrisi ile dönüştür
    return TransformPosition(Vector3.Zero, T);
}
```

### 3.4 Hesaplama Karmaşıklığı

- **Zaman Karmaşıklığı**: O(n) - n eklem sayısı (burada n=5)
- **Uzay Karmaşıklığı**: O(1) - Sabit matris boyutu

---

## 4. Ters Kinematik (Inverse Kinematics)

### 4.1 Problem Tanımı

Ters kinematik problemi: Verilen hedef pozisyon **p**ₜₐᵣ = [xₜ, yₜ, zₜ]ᵀ için, uç efektörün bu pozisyona ulaşmasını sağlayacak eklem açıları **q** = [θ₁, θ₂, θ₃, θ₄, θ₅]ᵀ vektörünü bulmak.

Bu problem genellikle:
- **Çok çözümlü** (multiple solutions)
- **Çözümsüz** (hedef erişim alanı dışında)
- **Analitik çözümü zor** (5+ DOF için)

### 4.2 Geometrik Analitik Çözüm Yaklaşımı

Bu implementasyon, 5-DOF problemi 3-DOF problemine indirgeyen geometrik bir yaklaşım kullanır.

#### Adım 1: Taban Açısı (θ₁) Hesabı

Hedef pozisyona üstten bakıldığında (XZ düzlemi):

```
θ₁ = atan2(xₜ, zₜ)
```

Hedefin tabana yatay uzaklığı:
```
r = √(xₜ² + zₜ²)
```

#### Adım 2: Düzlemsel 2-Link IK Problemi

Taban döndürüldükten sonra, problem 2D'ye indirgenmiş olur. Şimdi (r, y) düzleminde çalışırız.

**Basitleştirme**: Son iki linki birleştirerek 3-DOF sistemini 2-DOF'a indirge:
```
L₂' = L₂ = 0.8 (Üst kol)
L₃' = L₃ + L₄ = 0.6 + 0.5 = 1.1 (Alt kol toplam)
```

Omuz ekleminden hedef pozisyona mesafe:
```
y' = yₜ - L₁  (taban yüksekliği çıkarılır)
d = √(r² + y'²)
```

**Erişilebilirlik kontrolü:**
```
if (d > L₂' + L₃') → Hedef erişilemez, return false
```

#### Adım 3: Kosinüs Teoremi ile Dirsek Açısı

Üçgen geometrisinden kosinüs teoremi:

```
cos(θ₃) = (d² - L₂'² - L₃'²) / (2·L₂'·L₃')
```

Dirsek açısı:
```
θ₃ = acos(cos(θ₃))
```

**Not**: `cos(θ₃)` değeri [-1, 1] aralığında kısıtlanmalıdır (sayısal hata önleme).

#### Adım 4: Omuz Açısı (θ₂) Hesabı

İki bileşen açının toplamı:

**Hedefin açısal yönelimi:**
```
φ = atan2(y', r)
```

**İç açı düzeltmesi:**
```
ψ = atan2(L₃'·sin(θ₃), L₂' + L₃'·cos(θ₃))
```

**Nihai omuz açısı:**
```
θ₂ = π/2 - (φ + ψ)
```

**Not**: π/2 terimi, robotun dikey başlangıç duruşundan kaynaklanır.

#### Adım 5: Bilek Açılarının Dağıtımı

Hesaplanan toplam dirsek bükülmesini θ₃ ve θ₄ arasında dağıt:

```
θ₃_final = θ₃_total × 0.6  (Dirsek %60)
θ₄_final = θ₃_total × 0.4  (Bilek %40)
```

Son eklem sabit tutulur:
```
θ₅ = 0
```

### 4.3 Kod İmplementasyonu

```csharp
bool SolveIK(Vector3 target, out float θ₁, out float θ₂, 
             out float θ₃, out float θ₄, out float θ₅)
{
    // Adım 1: Taban açısı
    float t1 = atan2(target.X, target.Z);
    float r = sqrt(target.X² + target.Z²);
    
    // Adım 2: 2D problem
    float y = target.Y - L₁;
    float d = sqrt(r² + y²);
    
    float l1 = L₂;
    float l2 = L₃ + L₄;
    
    // Erişilebilirlik kontrolü
    if (d > l1 + l2) return false;
    
    // Adım 3: Kosinüs teoremi
    float cosQ3 = (d² - l1² - l2²) / (2·l1·l2);
    cosQ3 = Clamp(cosQ3, -1, 1);
    float q3 = acos(cosQ3);
    
    // Adım 4: Omuz açısı
    float phi = atan2(y, r);
    float psi = atan2(l2·sin(q3), l1 + l2·cos(q3));
    float q2 = π/2 - (phi + psi);
    
    // Adım 5: Açıları dağıt
    θ₁ = RadiansToDegrees(t1);
    θ₂ = RadiansToDegrees(q2);
    
    float totalElbow = RadiansToDegrees(q3);
    θ₃ = totalElbow × 0.6;
    θ₄ = totalElbow × 0.4;
    θ₅ = 0;
    
    return true;
}
```

### 4.4 Çözüm Özellikleri

- **Tip**: Geometrik analitik çözüm
- **Hız**: O(1) - Sabit zaman
- **Avantajlar**: 
  - Çok hızlı
  - Deterministik
  - Kapalı form çözüm
- **Dezavantajlar**:
  - Tek çözüm sağlar (birden fazla olası konfigürasyon varsa)
  - 5. eklem sabit (θ₅ = 0)
  - Eklem limitlerini tam kontrol etmez

---

## 5. Jacobian Matrisi ve Hız Kinematiği

### 5.1 Jacobian Matrisi Teorisi

Jacobian matrisi **J(q)**, eklem hızları **q̇** ile uç efektör hızı **ẋ** arasındaki ilişkiyi tanımlar:

```
ẋ = J(q) · q̇
```

Burada:
- **ẋ** = [ẋ, ẏ, ż]ᵀ ∈ ℝ³ : Kartezyen uzayda lineer hız
- **q̇** = [θ̇₁, θ̇₂, θ̇₃, θ̇₄, θ̇₅]ᵀ ∈ ℝ⁵ : Eklem açısal hızları
- **J(q)** ∈ ℝ³ˣ⁵ : Jacobian matrisi (3 satır × 5 sütun)

### 5.2 Jacobian Matris Yapısı

```
     ┌ ∂x/∂θ₁  ∂x/∂θ₂  ∂x/∂θ₃  ∂x/∂θ₄  ∂x/∂θ₅ ┐
J = │ ∂y/∂θ₁  ∂y/∂θ₂  ∂y/∂θ₃  ∂y/∂θ₄  ∂y/∂θ₅ │
     └ ∂z/∂θ₁  ∂z/∂θ₂  ∂z/∂θ₃  ∂z/∂θ₄  ∂z/∂θ₅ ┘
```

Her sütun j, j'inci eklemin uç efektör pozisyonu üzerindeki etkisini gösterir.

### 5.3 Nümerik Türev ile Jacobian Hesaplama

Analitik türev karmaşık olduğu için, nümerik yaklaşım kullanılır (sonlu farklar yöntemi):

**Merkezi Fark Yaklaşımı:**
```
∂f/∂x ≈ [f(x + h) - f(x)] / h
```

Her eklem için:
```
J[:, i] = [FK(θ₁,...,θᵢ+h,...,θ₅) - FK(θ₁,...,θᵢ,...,θ₅)] / h
```

Burada:
- **h** = 0.1° (türev adım boyutu)
- **FK()** = İleri kinematik fonksiyonu

### 5.4 Kod İmplementasyonu

```csharp
float[,] ComputeJacobian(float θ₁, float θ₂, float θ₃, float θ₄, float θ₅)
{
    float[,] J = new float[3, 5];  // 3×5 matris
    float h = 0.1°;  // Türev adımı
    
    Vector3 current = FK(θ₁, θ₂, θ₃, θ₄, θ₅);
    
    // Her sütun için (her eklem)
    for (int i = 0; i < 5; i++)
    {
        // i'inci açıyı h kadar artır
        Vector3 perturbed = FK_with_perturbation(i, h);
        
        // Nümerik türev
        Vector3 column = (perturbed - current) / Radians(h);
        
        J[0, i] = column.X;
        J[1, i] = column.Y;
        J[2, i] = column.Z;
    }
    
    return J;
}
```

### 5.5 Hız Kinematiği Hesaplama

Verilen eklem hızları için uç efektör hızını hesapla:

```csharp
Vector3 ComputeEndEffectorVelocity(float[,] J, float[] q_dot)
{
    // v = J · q̇
    Vector3 v = Vector3.Zero;
    
    for (int row = 0; row < 3; row++)      // X, Y, Z bileşenleri
    {
        for (int col = 0; col < 5; col++)  // 5 eklem
        {
            v[row] += J[row, col] * q_dot[col];
        }
    }
    
    return v;
}
```

**Hız büyüklüğü:**
```
|v| = √(vₓ² + vᵧ² + vᵤ²)
```

### 5.6 Jacobian'ın Önemi

Jacobian matrisi robot kontrolünde kritik öneme sahiptir:

1. **Hız Kontrolü**: Kartezyen uzayda hız hedefleri → Eklem hızları
2. **Kuvvet Kontrolü**: Eklem torkları → Uç efektör kuvveti
3. **Singülarite Analizi**: det(J·Jᵀ) ≈ 0 → Tekil konfigürasyon
4. **Ters Kinematik Optimizasyonu**: Nümerik IK çözümlerinde
5. **Yörünge Takibi**: Gerçek zamanlı yol kontrolü

### 5.7 Hesaplama Karmaşıklığı

- **Zaman**: O(n²) - n eklem sayısı (5 FK çağrısı × her biri O(n))
- **Uzay**: O(n) - 3×n matris depolama

---

## 6. Trajektori Planlama

### 6.1 Trajektori Planlama Nedir?

Trajektori planlama, robotun başlangıç konfigürasyonundan hedef konfigürasyona **yumuşak** ve **sürekli** bir şekilde geçişini sağlar. Aniden atlama yerine, zaman içinde kademeli değişim gerçekleştirilir.

### 6.2 Lineer İnterpolasyon (LERP)

En basit trajektori planlama yöntemi lineer interpolasyondur:

**Matematiksel Formül:**
```
q(t) = q_start + (q_target - q_start) · u(t)
```

Burada:
- **q(t)**: t anındaki eklem açıları
- **q_start**: Başlangıç açıları
- **q_target**: Hedef açıları
- **u(t)**: Normalizasyon parametresi [0, 1]

**Zaman normalizasyonu:**
```
u(t) = t / T_duration

u(t) ∈ [0, 1]  where:
  u(0) = 0     → Başlangıç
  u(T) = 1     → Bitiş
```

### 6.3 Eklem Uzayı vs Kartezyen Uzay

**Bu implementasyon: Eklem Uzayı (Joint Space)**

Avantajları:
- Her eklem bağımsız interpolasyon
- Eklem limitleri kolay kontrol
- Hesaplama basit ve hızlı
- Singülaritelerden etkilenmez

Dezavantajları:
- Uç efektör düz çizgi takip etmez
- Kartezyen uzayda öngörülemeyen yörünge

**Alternatif: Kartezyen Uzay**
- Uç efektör düz yol izler
- Her adımda IK çözümü gerekir
- Hesaplama yoğun
- Singülaritelerle karşılaşabilir

### 6.4 Kod İmplementasyonu

```csharp
// LERP Fonksiyonu
float Lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}

// Trajektori Güncellemesi (Her Frame)
void UpdateTrajectory(float deltaTime)
{
    if (!trajectoryActive) return;
    
    // Zamanı ilerlet
    trajectoryTime += deltaTime;
    
    // Normalizasyon parametresi
    float u = trajectoryTime / trajectoryDuration;
    
    // Bitiş kontrolü
    if (u >= 1.0f)
    {
        u = 1.0f;
        trajectoryActive = false;  // Trajektori tamamlandı
    }
    
    // Her eklem için LERP
    θ₁ = Lerp(θ₁_start, θ₁_target, u);
    θ₂ = Lerp(θ₂_start, θ₂_target, u);
    θ₃ = Lerp(θ₃_start, θ₃_target, u);
    θ₄ = Lerp(θ₄_start, θ₄_target, u);
    θ₅ = Lerp(θ₅_start, θ₅_target, u);
}
```

### 6.5 Gelişmiş Trajektori Profilleri

Lineer interpolasyon ani hız değişimine neden olur. Daha yumuşak profiller:

**S-Eğrisi (Sigmoid):**
```
u_smooth(t) = 3u² - 2u³
```

**Kosinüs Profili:**
```
u_smooth(t) = (1 - cos(πu)) / 2
```

**5. Derece Polinom:**
```
u_smooth(t) = 10u³ - 15u⁴ + 6u⁵
```

Bu profiller:
- Başlangıç ve bitişte **sıfır hız** garanti eder
- Sürekli ivme sağlar
- Mekanik sistemler için daha uygun

### 6.6 Zaman Parametreleri

Proje varsayılan değerleri:
```
T_duration = 2.0 saniye  (Toplam hareket süresi)
FPS = 60                  (Frame/saniye)
Toplam Frame = 120        (2.0 × 60)
```

Her frame'de açı değişimi:
```
Δθ = (θ_target - θ_start) / 120
```

---

## 7. 3D Görselleştirme

### 7.1 OpenGL Render Pipeline

Proje OpenTK (OpenGL) kullanarak gerçek zamanlı 3D render yapar.

**Render Adımları:**

1. **Clear Buffer**: Ekranı ve derinlik tamponunu temizle
2. **Projection Matrix**: Perspektif kamera ayarla
3. **View Matrix**: Kamera pozisyonu ve yönelimi
4. **Model Matrix**: Her obje için transformasyon
5. **Draw Calls**: Geometrik primitifler çiz
6. **Swap Buffer**: Çift tamponlama (double buffering)

### 7.2 Kamera Sistemi

**Perspektif Projeksiyon:**
```
Projection = CreatePerspective(
    FOV = 45°,           // Görüş alanı
    Aspect = W/H,        // En-boy oranı
    Near = 0.1,          // Yakın düzlem
    Far = 100.0          // Uzak düzlem
)
```

**Sferik Kamera Hareketi:**

Kamera pozisyonu polar koordinatlarda:
```
x = distance · cos(angleX) · cos(angleY)
y = distance · sin(angleX)
z = distance · cos(angleX) · sin(angleY)
```

Parametreler:
- **distance**: Merkeze uzaklık [2, 20]
- **angleX**: Dikey açı [-89°, +89°]
- **angleY**: Yatay açı [Sınırsız]

**View Matrix:**
```
ViewMatrix = LookAt(
    eye = cameraPosition,    // Kamera konumu
    target = (0, 0, 0),      // Baktığı nokta (orijin)
    up = (0, 1, 0)           // Yukarı yön vektörü
)
```

### 7.3 Işıklandırma Modeli (Phong Lighting)

OpenGL sabit fonksiyon hattı (fixed pipeline) ışıklandırması:

**Işık Bileşenleri:**
```
I_total = I_ambient + I_diffuse + I_specular
```

**Ambient (Ortam Işığı):**
```
I_ambient = K_a · L_a
K_a = 0.3  (Malzeme albedo)
```

**Diffuse (Yayınık Işık):**
```
I_diffuse = K_d · L_d · max(N · L, 0)
K_d = 1.0
L_d = (1, 1, 1)  (Beyaz ışık)
```

**Işık Pozisyonu:**
```
Light_position = (5, 10, 10, 1)
```

### 7.4 Geometri Çizimi

**Silindir Primitifi:**

Robot kolları silindir olarak modellenir (küp yerine daha gerçekçi):

```csharp
void DrawCylinder(float radius, float height, int segments)
{
    // Yan yüzey (QuadStrip)
    for (int i = 0; i <= segments; i++)
    {
        float angle = 2π · i / segments;
        float x = radius · cos(angle);
        float z = radius · sin(angle);
        
        // Normal vektör (ışıklandırma için)
        Normal(x, 0, z);
        
        // Üst ve alt vertex
        Vertex(x, height, z);
        Vertex(x, 0, z);
    }
    
    // Üst ve alt kapaklı
    DrawDisk(radius, 0);      // Alt kapak
    DrawDisk(radius, height);  // Üst kapak
}
```

**Boyutlar:**
- Link radius: 0.1 birim
- Joint radius: 0.18 birim
- Segment sayısı: 24 (pürüzsüz silindir)

### 7.5 Hiyerarşik Transformasyon

Robot çizimi ağaç yapısında (scene graph):

```
World
 └─ Base (θ₁)
     └─ Link1 (L₁)
         └─ Shoulder (θ₂)
             └─ Link2 (L₂)
                 └─ Elbow (θ₃)
                     └─ Link3 (L₃)
                         └─ Wrist (θ₄)
                             └─ Link4 (L₄)
                                 └─ WristRoll (θ₅)
                                     └─ Gripper
```

**OpenGL Matris Yığını (Matrix Stack):**
```csharp
PushMatrix();           // Mevcut matrisi kaydet
  Rotate(θ);            // Dönüş uygula
  Translate(0, L, 0);   // İleri git
  DrawLink();           // Link çiz
  DrawChild();          // Alt eleman çiz
PopMatrix();            // Önceki matrise dön
```

Bu yöntem:
- Ebeveyn transformasyonları alt elemanlara otomatik yayılır
- Lokal koordinat sistemleri kullanılır
- Kod temiz ve modüler olur

### 7.6 Renk Şeması

| Eleman | Renk | RGB |
|--------|------|-----|
| Linkler | Turuncu | (1.0, 0.5, 0.0) |
| Eklemler | Gri | (0.6, 0.6, 0.6) |
| Gripper Taban | Koyu Gri | (0.3, 0.3, 0.3) |
| Gripper Parmaklar | Açık Gri | (0.8, 0.8, 0.8) |
| Zemin | Koyu Gri | (0.2, 0.2, 0.2) |
| X Ekseni | Kırmızı | (1, 0, 0) |
| Y Ekseni | Yeşil | (0, 1, 0) |
| Z Ekseni | Mavi | (0, 0, 1) |

### 7.7 Gripper Animasyonu

Gripper 3 parmaktan oluşur (120° açılarda):

```
Parmak pozisyonu = 0.05 + (0.16 - 0.05) · t

t = (gap - gap_min) / (gap_max - gap_min)  [0, 1]
```

- **gap_min** = 0.03 (Kapalı)
- **gap_max** = 0.25 (Açık)

---

## 8. Kullanım Kılavuzu

### 8.1 Kurulum

**Gereksinimler:**
```
.NET Framework 4.7.2 Developer Pack
Visual Studio 2019/2022 veya VS Code
OpenTK 3.3.3 (NuGet ile otomatik)
```

**Derleme:**
```powershell
cd RobotArm5DOF
dotnet build
```

**Çalıştırma:**
```powershell
dotnet run
# veya
.\RobotArm5DOF\bin\Debug\RobotArm5DOF.exe
```

### 8.2 Klavye Kontrolleri

#### Eklem Kontrolleri

| Tuş | Fonksiyon | Hareket |
|-----|-----------|---------|
| Q / A | Taban (θ₁) | Sola/Sağa dönüş |
| W / S | Omuz (θ₂) | Öne/Arkaya eğilme |
| E / D | Dirsek (θ₃) | Kol bükülme |
| R / F | Bilek Pitch (θ₄) | Yukarı/Aşağı |
| T / G | Bilek Roll (θ₅) | Dönüş |

**Hareket hızı:** 60°/saniye

#### Gripper Kontrolü

| Tuş | Fonksiyon |
|-----|-----------|
| X | Gripper Aç |
| Z | Gripper Kapat |

**Hareket hızı:** 0.6 birim/saniye

#### Kamera Kontrolleri

| Tuş | Fonksiyon |
|-----|-----------|
| ← / → | Yatay dönüş (angleY) |
| ↑ / ↓ | Dikey dönüş (angleX) |
| Page Up | Yakınlaş (Zoom In) |
| Page Down | Uzaklaş (Zoom Out) |

**Kamera limitleri:**
- Dikey: [-89°, +89°]
- Uzaklık: [2, 20] birim

#### Ters Kinematik

| Tuş | Fonksiyon | Açıklama |
|-----|-----------|----------|
| I | IK Anında | Hedef pozisyona anında git |
| Y | IK Yumuşak | Hedef pozisyona trajektori ile git (2 saniye) |

**Hedef pozisyon:** (1.2, 1.5, 0.4)

#### Genel

| Tuş | Fonksiyon |
|-----|-----------|
| ESC | Çıkış |

### 8.3 Pencere Başlığı Bilgileri

Program başlığında gerçek zamanlı bilgiler gösterilir:

```
5DOF Robot | Aci:(45,30,-20,15,0) | EE:(1.23,2.45,0.67) | |Vee|:0.34
```

- **Aci**: Mevcut eklem açıları [θ₁, θ₂, θ₃, θ₄, θ₅]
- **EE**: Uç efektör pozisyonu [x, y, z]
- **|Vee|**: Uç efektör hız büyüklüğü (varsayılan eklem hızları için)

---

## 9. Teknik Detaylar

### 9.1 Performans Özellikleri

**Hesaplama Yükü (Her Frame):**

| İşlem | Karmaşıklık | Çağrı Sayısı | Süre (yaklaşık) |
|-------|-------------|--------------|-----------------|
| Forward Kinematics | O(5) | 7 | < 0.1 ms |
| Jacobian | O(25) | 1 | < 0.5 ms |
| Rendering | O(N) | 1 | 1-2 ms |
| **Toplam** | | | **< 3 ms** |

**Frame Rate:** 60 FPS (16.67 ms/frame) → Yeterli marj

### 9.2 Sayısal Stabilite

**Potansiyel Problemler:**

1. **Ters Kinematik:**
   - Acos domain hatası: `cos(θ) ∈ [-1, 1]` kontrolü
   - Çözüm: `Clamp(value, -1, 1)`

2. **Jacobian Hesaplama:**
   - Sıfıra bölme: h değeri çok küçük olmamalı
   - Çözüm: h = 0.1° (yeterince büyük)

3. **Gimbal Lock:**
   - X ekseni dönüşleri art arda → Potansiyel tekil nokta
   - Çözüm: Eklem limitlerle kontrol

**Sayısal Hassasiyet:**
- Float (32-bit): ~7 ondalık basamak
- Açı çözünürlüğü: 0.01° (yeterli)

### 9.3 Koordinat Sistemi Uyumu

**Kritik**: DrawRobot() ve ComputeForwardKinematics() fonksiyonları **birebir** aynı transformasyon sırasını kullanmalıdır.

Aksi halde:
- Görsel ve hesaplanan pozisyon uyumsuz olur
- IK çözümü hatalı hedeflere gider
- Jacobian matrisi yanlış hesaplanır

**Doğrulama Yöntemi:**
Herhangi bir eklem konfigürasyonu için:
```
visual_position ≈ FK(θ₁, θ₂, θ₃, θ₄, θ₅)
```

### 9.4 Eklem Limitleri

```csharp
// Fiziksel limitler (derece cinsinden)
θ₂ ∈ [-90, +90]    // Omuz
θ₃ ∈ [-150, +150]  // Dirsek
θ₄ ∈ [-90, +90]    // Bilek
```

**Neden gerekli:**
- Fiziksel robotlarda mekanik limitler vardır
- Self-collision (kendi kendine çarpma) önler
- Gerçekçi hareket aralığı

**Limit Kontrolü:**
```csharp
θ₂ = Clamp(θ₂, -90, 90);
```

### 9.5 Singülarite (Tekil Noktalar)

**Singülarite Nedir?**

Robot belirli konfigürasyonlarda serbestlik kaybeder. Jacobian matrisinin determinantı sıfır olur.

**Bu Sistemde Potansiyel Singülariteler:**

1. **Taban Singülaritesi:**
   - Hedef orijin üzerinde (r = 0)
   - θ₁ belirsiz hale gelir
   - Çözüm: Minimal r eşiği

2. **Kol Tam Uzandığında:**
   - d = L₂ + L₃ + L₄
   - θ₃ ≈ 0 (Kol düz)
   - Küçük hareket büyük eklem değişimi gerektirir

3. **Kol Tam Büküldüğünde:**
   - θ₃ ≈ ±150°
   - Bilek ve dirsek üst üste

**Singülarite Kontrolü:**
```
det(J·Jᵀ) < ε  →  Singülarite yakın
```

Burada ε küçük eşik değeridir (örn. 0.001).


## 10. Matematiksel Referanslar

### 10.1 Kullanılan Formüller Özeti

| Kavram | Formül | Bölüm |
|--------|--------|-------|
| Forward Kinematics | **p** = FK(**q**) = T₀⁵ · [0,0,0,1]ᵀ | 3 |
| Inverse Kinematics | θ₁ = atan2(x, z) | 4 |
| Kosinüs Teoremi | cos(C) = (a² + b² - c²) / (2ab) | 4 |
| Jacobian | J[i,j] = ∂pᵢ/∂θⱼ | 5 |
| Nümerik Türev | f'(x) ≈ [f(x+h) - f(x)] / h | 5 |
| Hız Kinematiği | **v** = J(**q**) · **q̇** | 5 |
| LERP | q(t) = q₀ + (q₁ - q₀) · t | 6 |
| Sferik Kamera | x = d·cos(φ)·cos(θ) | 7 |

### 10.2 Notasyon Tablosu

| Sembol | Anlamı | Birim |
|--------|--------|-------|
| θᵢ | i'inci eklem açısı | derece veya radyan |
| **q** | Eklem açıları vektörü [θ₁,...,θ₅]ᵀ | - |
| **p** | Kartezyen pozisyon [x,y,z]ᵀ | birim |
| Lᵢ | i'inci link uzunluğu | birim |
| **J** | Jacobian matrisi (3×5) | birim/radyan |
| **v** | Lineer hız vektörü | birim/saniye |
| **q̇** | Eklem açısal hızları | radyan/saniye |
| T | Homojen transformasyon matrisi (4×4) | - |

### 10.3 Koordinat Sistemleri

**Dünya Koordinatları (World Frame):**
- Orijin: Robot tabanı
- Y ekseni: Yukarı (gravitasyon tersi)
**Eklem Koordinatları (Joint Frame):**
- Her eklem lokal koordinat sistemine sahip
- Dönüş ekseni: X veya Y

**Uç Efektör Koordinatları (End-Effector Frame):**
- Son eklem merkezinde
- Oryantasyon θ₅ ile belirlenir

---

## 11. Kaynakça ve İleri Okuma

### 11.1 Robotik Temel Kitaplar

1. **Craig, J.J.** (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson Education.
   - Kinematik ve dinamik temel referans kitabı
   - Forward/Inverse Kinematics ve Jacobian detaylı anlatım

2. **Spong, M.W., Hutchinson, S., & Vidyasagar, M.** (2020). *Robot Modeling and Control* (2nd ed.). Wiley.
   - DH parametreleri ve transformasyonlar
   - Robot kontrol algoritmaları

3. **Lynch, K.M., & Park, F.C.** (2017). *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press.
   - Modern yaklaşımlar ve screw theory
   - **[Ücretsiz online: modernrobotics.org](http://modernrobotics.org)**

4. **Siciliano, B., Sciavicco, L., Villani, L., & Oriolo, G.** (2010). *Robotics: Modelling, Planning and Control*. Springer.
   - İleri seviye kontrol teorisi
   - Yörünge planlama ve manipülasyon

5. **Corke, P.I.** (2017). *Robotics, Vision and Control: Fundamental Algorithms in MATLAB* (2nd ed.). Springer.
   - Pratik MATLAB implementasyonları
   - Robot kütüphanesi örnekleri

### 11.2 Bilgisayar Grafikleri

6. **Shirley, P., & Marschner, S.** (2009). *Fundamentals of Computer Graphics* (3rd ed.). AK Peters/CRC Press.
   - Transformasyon matrisleri ve render pipeline
   - 3D grafik matematiği

7. **Shreiner, D., et al.** (2013). *OpenGL Programming Guide* (8th ed.). Addison-Wesley.
   - OpenGL API referansı (Red Book)
   - Shader programlama temelleri

### 11.3 Matematiksel Temeller

8. **Strang, G.** (2016). *Introduction to Linear Algebra* (5th ed.). Wellesley-Cambridge Press.
   - Matris işlemleri ve lineer dönüşümler
   - MIT OpenCourseWare dersleriyle destekli

9. **Press, W.H., et al.** (2007). *Numerical Recipes: The Art of Scientific Computing* (3rd ed.). Cambridge University Press.
   - Nümerik türev ve optimizasyon
   - Pratik algoritma implementasyonları

### 11.4 C# ve .NET Programlama

10. **Albahari, J., & Albahari, B.** (2021). *C# 10 in a Nutshell*. O'Reilly Media.
    - C# dil özellikleri ve .NET framework
    - LINQ ve async programlama

### 11.5 Online Kaynaklar

11. **[OpenTK Documentation](https://opentk.net/)**
    - OpenTK API tam referansı

12. **[Learn OpenGL](https://learnopengl.com/)**
    - Modern OpenGL comprehensive tutorial

13. **[Robot Academy - QUT](https://robotacademy.net.au/)**
    - Peter Corke'un video ders serisi
    - Ücretsiz robotik eğitimi

14. **[Stanford CS223A - Introduction to Robotics](https://see.stanford.edu/Course/CS223A)**
    - Prof. Oussama Khatib video dersleri
    - Kinematik ve dinamik detaylı anlatım

15. **[IEEE Robotics and Automation Society](https://www.ieee-ras.org/)**
    - Akademik makaleler ve konferanslar
    - Robotik araştırma trendleri

---

## Ekler

### A. Kod Yapısı

```
RobotArm5DOF/
├── Program.cs              (Ana kod - 617 satır)
│   ├── Kinematic Variables (θ₁-θ₅, L₁-L₄)
│   ├── OnLoad()            (OpenGL başlatma)
│   ├── OnUpdateFrame()     (Fizik ve input)
│   ├── OnRenderFrame()     (Render döngüsü)
│   ├── DrawRobot()         (Robot çizimi)
│   ├── DrawCylinder()      (Geometri)
│   ├── ComputeForwardKinematics()
│   ├── SolveIK()           (Ters kinematik)
│   ├── ComputeJacobian()   (Jacobian matrisi)
│   └── Lerp()              (Trajektori)
├── RobotArm5DOF.csproj
├── packages.config
└── README.md               (Bu dosya)
```

### B. Hızlı Başvuru Kartı

**Eklem Kontrol:**
```
Q/A: Taban (θ₁)     W/S: Omuz (θ₂)      E/D: Dirsek (θ₃)
R/F: Bilek (θ₄)     T/G: Roll (θ₅)      X/Z: Gripper
```

**Kamera:**
```
Oklar: Döndür       PgUp/PgDn: Zoom     ESC: Çıkış
```

**IK:**
```
I: Anında git       Y: Yumuşak git
```

---

<a id="english-version"></a>

# 5 Degrees of Freedom (5-DOF) Robot Arm Simulation

> **Language / Dil:** [🇹🇷 Türkçe](#5-serbestlik-dereceli-5-dof-robot-kol-simülasyonu) | [🇬🇧 English](#english-version)

## Table of Contents
1. [Project Overview](#1-project-overview)
2. [Kinematic Structure and Joint Configuration](#2-kinematic-structure-and-joint-configuration)
3. [Forward Kinematics](#3-forward-kinematics)
4. [Inverse Kinematics](#4-inverse-kinematics)
5. [Jacobian Matrix and Velocity Kinematics](#5-jacobian-matrix-and-velocity-kinematics)
6. [Trajectory Planning](#6-trajectory-planning)
7. [3D Visualization](#7-3d-visualization)
8. [User Guide](#8-user-guide)
9. [Technical Details](#9-technical-details)

---

## 1. Project Overview

This project implements a real-time simulation of a 5 degrees of freedom (5-DOF) robot arm using OpenGL and C#. The project encompasses fundamental kinematic concepts in robotics:

- **Forward Kinematics (FK)**: Computing end-effector position from joint angles
- **Inverse Kinematics (IK)**: Computing joint angles from target position
- **Jacobian Matrix**: Relationship between joint velocities and end-effector velocity
- **Trajectory Planning**: Smooth motion transitions
- **3D Visualization**: Real-time robot simulation

### Technology Stack
- **Platform**: .NET Framework 4.7.2
- **Graphics Library**: OpenTK 3.3.3 (OpenGL)
- **Language**: C# 
- **IDE**: Visual Studio / Visual Studio Code

---

## 2. Kinematic Structure and Joint Configuration

### 2.1 Joint Definitions

The robot arm consists of 5 revolute joints:

| Joint | Name | Rotation Axis | Angle Variable | Motion Limits | Description |
|-------|------|---------------|----------------|---------------|-------------|
| J1 | Base | Y axis | θ₁ | Unlimited | Base rotation (Yaw) |
| J2 | Shoulder | X axis | θ₂ | [-90°, +90°] | Forward/backward tilt (Pitch) |
| J3 | Elbow | X axis | θ₃ | [-150°, +150°] | Arm bending (Pitch) |
| J4 | Wrist Pitch | X axis | θ₄ | [-90°, +90°] | Wrist up/down (Pitch) |
| J5 | Wrist Roll | Y axis | θ₅ | Unlimited | End-effector rotation (Yaw) |

### 2.2 Link Lengths

Physical parameters of the robot arm:

```
L₁ = 1.0 unit  (Base height)
L₂ = 0.8 unit  (Shoulder-Elbow)
L₃ = 0.6 unit  (Elbow-Wrist)
L₄ = 0.5 unit  (Wrist-End)
```

**Total Maximum Reach**: L₁ + L₂ + L₃ + L₄ = 2.9 units

### 2.3 Coordinate System

The project uses the standard robotics coordinate system:
- **X axis**: Red - Horizontal (left/right)
- **Y axis**: Green - Vertical (up/down)
- **Z axis**: Blue - Depth (forward/backward)

Right-hand rule is applied.

---

## 3. Forward Kinematics

### 3.1 Mathematical Formulation

Forward kinematics computes the end-effector position **p** = [x, y, z]ᵀ given joint angles vector **q** = [θ₁, θ₂, θ₃, θ₄, θ₅]ᵀ.

#### Direct Matrix Multiplication Instead of Denavit-Hartenberg (DH) Approach

This implementation uses homogeneous transformation matrices directly instead of DH parameters:

```
T₀⁵ = T₀¹ · T¹² · T²³ · T³⁴ · T⁴⁵
```

Transformation for each joint:

**Joint 1 (Base - Y axis rotation):**
```
T₀¹ = Rot(Y, θ₁) · Trans(0, L₁, 0)
```

**Joint 2 (Shoulder - X axis rotation):**
```
T¹² = Rot(X, θ₂) · Trans(0, L₂, 0)
```

**Joint 3 (Elbow - X axis rotation):**
```
T²³ = Rot(X, θ₃) · Trans(0, L₃, 0)
```

**Joint 4 (Wrist Pitch - X axis rotation):**
```
T³⁴ = Rot(X, θ₄) · Trans(0, L₄, 0)
```

**Joint 5 (Wrist Roll - Y axis rotation):**
```
T⁴⁵ = Rot(Y, θ₅)
```

### 3.2 Homogeneous Transformation Matrices

**Rotation around Y axis:**
```
       ┌ cos(θ)   0   sin(θ)   0 ┐
Rʸ(θ) =│   0      1     0      0 │
       │-sin(θ)   0   cos(θ)   0 │
       └   0      0     0      1 ┘
```

**Rotation around X axis:**
```
       ┌  1      0       0      0 ┐
Rˣ(θ) =│  0   cos(θ) -sin(θ)   0 │
       │  0   sin(θ)  cos(θ)   0 │
       └  0      0       0      1 ┘
```

**Translation matrix:**
```
          ┌ 1  0  0  tₓ ┐
Trans(t) =│ 0  1  0  tᵧ │
          │ 0  0  1  tᵤ │
          └ 0  0  0  1  ┘
```

### 3.3 Code Implementation

```csharp
Vector3 ComputeForwardKinematics(float θ₁, float θ₂, float θ₃, float θ₄, float θ₅)
{
    // Degree -> Radian conversion
    float t1 = DegreesToRadians(θ₁);
    float t2 = DegreesToRadians(θ₂);
    float t3 = DegreesToRadians(θ₃);
    float t4 = DegreesToRadians(θ₄);
    float t5 = DegreesToRadians(θ₅);

    Matrix4 T = Identity;
    
    // Apply joint transformations in sequence
    T *= RotationY(t1) * Translation(0, L₁, 0);
    T *= RotationX(t2) * Translation(0, L₂, 0);
    T *= RotationX(t3) * Translation(0, L₃, 0);
    T *= RotationX(t4) * Translation(0, L₄, 0);
    T *= RotationY(t5);
    
    // Transform origin point with transformation matrix
    return TransformPosition(Vector3.Zero, T);
}
```

### 3.4 Computational Complexity

- **Time Complexity**: O(n) - n is number of joints (here n=5)
- **Space Complexity**: O(1) - Constant matrix size

---

## 4. Inverse Kinematics

### 4.1 Problem Definition

The inverse kinematics problem: Given target position **p**ₜₐᵣ = [xₜ, yₜ, zₜ]ᵀ, find the joint angles vector **q** = [θ₁, θ₂, θ₃, θ₄, θ₅]ᵀ that brings the end-effector to this position.

This problem is typically:
- **Multiple solutions** (multiple possible configurations)
- **No solution** (target outside reachable workspace)
- **Difficult analytical solution** (for 5+ DOF)

### 4.2 Geometric Analytical Solution Approach

This implementation uses a geometric approach that reduces the 5-DOF problem to a 3-DOF problem.

#### Step 1: Base Angle (θ₁) Calculation

Looking at target position from top (XZ plane):

```
θ₁ = atan2(xₜ, zₜ)
```

Horizontal distance to target from base:
```
r = √(xₜ² + zₜ²)
```

#### Step 2: Planar 2-Link IK Problem

After rotating the base, the problem is reduced to 2D. Now we work in (r, y) plane.

**Simplification**: Combine last two links to reduce 3-DOF system to 2-DOF:
```
L₂' = L₂ = 0.8 (Upper arm)
L₃' = L₃ + L₄ = 0.6 + 0.5 = 1.1 (Total lower arm)
```

Distance from shoulder joint to target position:
```
y' = yₜ - L₁  (base height subtracted)
d = √(r² + y'²)
```

**Reachability check:**
```
if (d > L₂' + L₃') → Target unreachable, return false
```

#### Step 3: Elbow Angle Using Law of Cosines

From triangle geometry using law of cosines:

```
cos(θ₃) = (d² - L₂'² - L₃'²) / (2·L₂'·L₃')
```

Elbow angle:
```
θ₃ = acos(cos(θ₃))
```

**Note**: `cos(θ₃)` value must be clamped to [-1, 1] (numerical error prevention).

#### Step 4: Shoulder Angle (θ₂) Calculation

Sum of two component angles:

**Angular orientation of target:**
```
φ = atan2(y', r)
```

**Internal angle correction:**
```
ψ = atan2(L₃'·sin(θ₃), L₂' + L₃'·cos(θ₃))
```

**Final shoulder angle:**
```
θ₂ = π/2 - (φ + ψ)
```

**Note**: The π/2 term comes from the robot's vertical starting posture.

#### Step 5: Distribution of Wrist Angles

Distribute calculated total elbow bending between θ₃ and θ₄:

```
θ₃_final = θ₃_total × 0.6  (Elbow 60%)
θ₄_final = θ₃_total × 0.4  (Wrist 40%)
```

Last joint kept fixed:
```
θ₅ = 0
```

### 4.3 Code Implementation

```csharp
bool SolveIK(Vector3 target, out float θ₁, out float θ₂, 
             out float θ₃, out float θ₄, out float θ₅)
{
    // Step 1: Base angle
    float t1 = atan2(target.X, target.Z);
    float r = sqrt(target.X² + target.Z²);
    
    // Step 2: 2D problem
    float y = target.Y - L₁;
    float d = sqrt(r² + y²);
    
    float l1 = L₂;
    float l2 = L₃ + L₄;
    
    // Reachability check
    if (d > l1 + l2) return false;
    
    // Step 3: Law of cosines
    float cosQ3 = (d² - l1² - l2²) / (2·l1·l2);
    cosQ3 = Clamp(cosQ3, -1, 1);
    float q3 = acos(cosQ3);
    
    // Step 4: Shoulder angle
    float phi = atan2(y, r);
    float psi = atan2(l2·sin(q3), l1 + l2·cos(q3));
    float q2 = π/2 - (phi + psi);
    
    // Step 5: Distribute angles
    θ₁ = RadiansToDegrees(t1);
    θ₂ = RadiansToDegrees(q2);
    
    float totalElbow = RadiansToDegrees(q3);
    θ₃ = totalElbow × 0.6;
    θ₄ = totalElbow × 0.4;
    θ₅ = 0;
    
    return true;
}
```

### 4.4 Solution Properties

- **Type**: Geometric analytical solution
- **Speed**: O(1) - Constant time
- **Advantages**: 
  - Very fast
  - Deterministic
  - Closed-form solution
- **Disadvantages**:
  - Provides single solution (if multiple configurations possible)
  - 5th joint fixed (θ₅ = 0)
  - Does not fully check joint limits

---

## 5. Jacobian Matrix and Velocity Kinematics

### 5.1 Jacobian Matrix Theory

The Jacobian matrix **J(q)** defines the relationship between joint velocities **q̇** and end-effector velocity **ẋ**:

```
ẋ = J(q) · q̇
```

Where:
- **ẋ** = [ẋ, ẏ, ż]ᵀ ∈ ℝ³ : Linear velocity in Cartesian space
- **q̇** = [θ̇₁, θ̇₂, θ̇₃, θ̇₄, θ̇₅]ᵀ ∈ ℝ⁵ : Joint angular velocities
- **J(q)** ∈ ℝ³ˣ⁵ : Jacobian matrix (3 rows × 5 columns)

### 5.2 Jacobian Matrix Structure

```
     ┌ ∂x/∂θ₁  ∂x/∂θ₂  ∂x/∂θ₃  ∂x/∂θ₄  ∂x/∂θ₅ ┐
J = │ ∂y/∂θ₁  ∂y/∂θ₂  ∂y/∂θ₃  ∂y/∂θ₄  ∂y/∂θ₅ │
     └ ∂z/∂θ₁  ∂z/∂θ₂  ∂z/∂θ₃  ∂z/∂θ₄  ∂z/∂θ₅ ┘
```

Each column j shows the effect of the jth joint on end-effector position.

### 5.3 Numerical Jacobian Calculation Using Finite Differences

Since analytical derivatives are complex, numerical approximation is used (finite difference method):

**Central Difference Approximation:**
```
∂f/∂x ≈ [f(x + h) - f(x)] / h
```

For each joint:
```
J[:, i] = [FK(θ₁,...,θᵢ+h,...,θ₅) - FK(θ₁,...,θᵢ,...,θ₅)] / h
```

Where:
- **h** = 0.1° (derivative step size)
- **FK()** = Forward kinematics function

### 5.4 Code Implementation

```csharp
float[,] ComputeJacobian(float θ₁, float θ₂, float θ₃, float θ₄, float θ₅)
{
    float[,] J = new float[3, 5];  // 3×5 matrix
    float h = 0.1°;  // Derivative step
    
    Vector3 current = FK(θ₁, θ₂, θ₃, θ₄, θ₅);
    
    // For each column (each joint)
    for (int i = 0; i < 5; i++)
    {
        // Perturb ith angle by h
        Vector3 perturbed = FK_with_perturbation(i, h);
        
        // Numerical derivative
        Vector3 column = (perturbed - current) / Radians(h);
        
        J[0, i] = column.X;
        J[1, i] = column.Y;
        J[2, i] = column.Z;
    }
    
    return J;
}
```

### 5.5 Velocity Kinematics Calculation

Calculate end-effector velocity for given joint velocities:

```csharp
Vector3 ComputeEndEffectorVelocity(float[,] J, float[] q_dot)
{
    // v = J · q̇
    Vector3 v = Vector3.Zero;
    
    for (int row = 0; row < 3; row++)      // X, Y, Z components
    {
        for (int col = 0; col < 5; col++)  // 5 joints
        {
            v[row] += J[row, col] * q_dot[col];
        }
    }
    
    return v;
}
```

**Velocity magnitude:**
```
|v| = √(vₓ² + vᵧ² + vᵤ²)
```

### 5.6 Importance of Jacobian

The Jacobian matrix is critical in robot control:

1. **Velocity Control**: Cartesian space velocity targets → Joint velocities
2. **Force Control**: Joint torques → End-effector force
3. **Singularity Analysis**: det(J·Jᵀ) ≈ 0 → Singular configuration
4. **Inverse Kinematics Optimization**: In numerical IK solutions
5. **Trajectory Tracking**: Real-time path control

### 5.7 Computational Complexity

- **Time**: O(n²) - n number of joints (5 FK calls × each O(n))
- **Space**: O(n) - 3×n matrix storage

---

## 6. Trajectory Planning

### 6.1 What is Trajectory Planning?

Trajectory planning ensures the robot transitions from start configuration to target configuration in a **smooth** and **continuous** manner. Instead of sudden jumps, gradual change occurs over time.

### 6.2 Linear Interpolation (LERP)

The simplest trajectory planning method is linear interpolation:

**Mathematical Formula:**
```
q(t) = q_start + (q_target - q_start) · u(t)
```

Where:
- **q(t)**: Joint angles at time t
- **q_start**: Starting angles
- **q_target**: Target angles
- **u(t)**: Normalization parameter [0, 1]

**Time normalization:**
```
u(t) = t / T_duration

u(t) ∈ [0, 1]  where:
  u(0) = 0     → Start
  u(T) = 1     → End
```

### 6.3 Joint Space vs Cartesian Space

**This implementation: Joint Space**

Advantages:
- Independent interpolation for each joint
- Easy joint limit control
- Simple and fast computation
- Unaffected by singularities

Disadvantages:
- End-effector doesn't follow straight line
- Unpredictable trajectory in Cartesian space

**Alternative: Cartesian Space**
- End-effector follows straight path
- Requires IK solution at each step
- Computationally intensive
- May encounter singularities

### 6.4 Code Implementation

```csharp
// LERP Function
float Lerp(float a, float b, float t)
{
    return a + (b - a) * t;
}

// Trajectory Update (Every Frame)
void UpdateTrajectory(float deltaTime)
{
    if (!trajectoryActive) return;
    
    // Advance time
    trajectoryTime += deltaTime;
    
    // Normalization parameter
    float u = trajectoryTime / trajectoryDuration;
    
    // End check
    if (u >= 1.0f)
    {
        u = 1.0f;
        trajectoryActive = false;  // Trajectory complete
    }
    
    // LERP for each joint
    θ₁ = Lerp(θ₁_start, θ₁_target, u);
    θ₂ = Lerp(θ₂_start, θ₂_target, u);
    θ₃ = Lerp(θ₃_start, θ₃_target, u);
    θ₄ = Lerp(θ₄_start, θ₄_target, u);
    θ₅ = Lerp(θ₅_start, θ₅_target, u);
}
```

### 6.5 Advanced Trajectory Profiles

Linear interpolation causes sudden velocity changes. Smoother profiles:

**S-Curve (Sigmoid):**
```
u_smooth(t) = 3u² - 2u³
```

**Cosine Profile:**
```
u_smooth(t) = (1 - cos(πu)) / 2
```

**5th Order Polynomial:**
```
u_smooth(t) = 10u³ - 15u⁴ + 6u⁵
```

These profiles:
- Guarantee **zero velocity** at start and end
- Provide continuous acceleration
- More suitable for mechanical systems

### 6.6 Time Parameters

Project default values:
```
T_duration = 2.0 seconds  (Total motion duration)
FPS = 60                  (Frames/second)
Total Frames = 120        (2.0 × 60)
```

Angle change per frame:
```
Δθ = (θ_target - θ_start) / 120
```

---

## 7. 3D Visualization

### 7.1 OpenGL Render Pipeline

The project performs real-time 3D rendering using OpenTK (OpenGL).

**Render Steps:**

1. **Clear Buffer**: Clear screen and depth buffer
2. **Projection Matrix**: Set up perspective camera
3. **View Matrix**: Camera position and orientation
4. **Model Matrix**: Transformation for each object
5. **Draw Calls**: Draw geometric primitives
6. **Swap Buffer**: Double buffering

### 7.2 Camera System

**Perspective Projection:**
```
Projection = CreatePerspective(
    FOV = 45°,           // Field of view
    Aspect = W/H,        // Aspect ratio
    Near = 0.1,          // Near plane
    Far = 100.0          // Far plane
)
```

**Spherical Camera Movement:**

Camera position in polar coordinates:
```
x = distance · cos(angleX) · cos(angleY)
y = distance · sin(angleX)
z = distance · cos(angleX) · sin(angleY)
```

Parameters:
- **distance**: Distance to center [2, 20]
- **angleX**: Vertical angle [-89°, +89°]
- **angleY**: Horizontal angle [Unlimited]

**View Matrix:**
```
ViewMatrix = LookAt(
    eye = cameraPosition,    // Camera location
    target = (0, 0, 0),      // Look-at point (origin)
    up = (0, 1, 0)           // Up direction vector
)
```

### 7.3 Lighting Model (Phong Lighting)

OpenGL fixed function pipeline lighting:

**Light Components:**
```
I_total = I_ambient + I_diffuse + I_specular
```

**Ambient (Ambient Light):**
```
I_ambient = K_a · L_a
K_a = 0.3  (Material albedo)
```

**Diffuse (Diffuse Light):**
```
I_diffuse = K_d · L_d · max(N · L, 0)
K_d = 1.0
L_d = (1, 1, 1)  (White light)
```

**Light Position:**
```
Light_position = (5, 10, 10, 1)
```

### 7.4 Geometry Drawing

**Cylinder Primitive:**

Robot arms are modeled as cylinders (more realistic than cubes):

```csharp
void DrawCylinder(float radius, float height, int segments)
{
    // Side surface (QuadStrip)
    for (int i = 0; i <= segments; i++)
    {
        float angle = 2π · i / segments;
        float x = radius · cos(angle);
        float z = radius · sin(angle);
        
        // Normal vector (for lighting)
        Normal(x, 0, z);
        
        // Top and bottom vertex
        Vertex(x, height, z);
        Vertex(x, 0, z);
    }
    
    // Top and bottom caps
    DrawDisk(radius, 0);      // Bottom cap
    DrawDisk(radius, height);  // Top cap
}
```

**Dimensions:**
- Link radius: 0.1 unit
- Joint radius: 0.18 unit
- Segment count: 24 (smooth cylinder)

### 7.5 Hierarchical Transformation

Robot drawing in tree structure (scene graph):

```
World
 └─ Base (θ₁)
     └─ Link1 (L₁)
         └─ Shoulder (θ₂)
             └─ Link2 (L₂)
                 └─ Elbow (θ₃)
                     └─ Link3 (L₃)
                         └─ Wrist (θ₄)
                             └─ Link4 (L₄)
                                 └─ WristRoll (θ₅)
                                     └─ Gripper
```

**OpenGL Matrix Stack:**
```csharp
PushMatrix();           // Save current matrix
  Rotate(θ);            // Apply rotation
  Translate(0, L, 0);   // Move forward
  DrawLink();           // Draw link
  DrawChild();          // Draw child element
PopMatrix();            // Restore previous matrix
```

This method:
- Parent transformations automatically propagate to children
- Uses local coordinate systems
- Clean and modular code

### 7.6 Color Scheme

| Element | Color | RGB |
|---------|-------|-----|
| Links | Orange | (1.0, 0.5, 0.0) |
| Joints | Gray | (0.6, 0.6, 0.6) |
| Gripper Base | Dark Gray | (0.3, 0.3, 0.3) |
| Gripper Fingers | Light Gray | (0.8, 0.8, 0.8) |
| Ground | Dark Gray | (0.2, 0.2, 0.2) |
| X Axis | Red | (1, 0, 0) |
| Y Axis | Green | (0, 1, 0) |
| Z Axis | Blue | (0, 0, 1) |

### 7.7 Gripper Animation

Gripper consists of 3 fingers (120° apart):

```
Finger position = 0.05 + (0.16 - 0.05) · t

t = (gap - gap_min) / (gap_max - gap_min)  [0, 1]
```

- **gap_min** = 0.03 (Closed)
- **gap_max** = 0.25 (Open)

---

## 8. User Guide

### 8.1 Installation

**Requirements:**
```
.NET Framework 4.7.2 Developer Pack
Visual Studio 2019/2022 or VS Code
OpenTK 3.3.3 (automatic via NuGet)
```

**Build:**
```powershell
cd RobotArm5DOF
dotnet build
```

**Run:**
```powershell
dotnet run
# or
.\RobotArm5DOF\bin\Debug\RobotArm5DOF.exe
```

### 8.2 Keyboard Controls

#### Joint Controls

| Key | Function | Motion |
|-----|----------|--------|
| Q / A | Base (θ₁) | Left/Right rotation |
| W / S | Shoulder (θ₂) | Forward/Backward tilt |
| E / D | Elbow (θ₃) | Arm bending |
| R / F | Wrist Pitch (θ₄) | Up/Down |
| T / G | Wrist Roll (θ₅) | Rotation |

**Motion speed:** 60°/second

#### Gripper Control

| Key | Function |
|-----|----------|
| X | Open Gripper |
| Z | Close Gripper |

**Motion speed:** 0.6 units/second

#### Camera Controls

| Key | Function |
|-----|----------|
| ← / → | Horizontal rotation (angleY) |
| ↑ / ↓ | Vertical rotation (angleX) |
| Page Up | Zoom In |
| Page Down | Zoom Out |

**Camera limits:**
- Vertical: [-89°, +89°]
- Distance: [2, 20] units

#### Inverse Kinematics

| Key | Function | Description |
|-----|----------|-------------|
| I | IK Instant | Go to target position instantly |
| Y | IK Smooth | Go to target position with trajectory (2 seconds) |

**Target position:** (1.2, 1.5, 0.4)

#### General

| Key | Function |
|-----|----------|
| ESC | Exit |

### 8.3 Window Title Information

Real-time information is displayed in the program title:

```
5DOF Robot | Ang:(45,30,-20,15,0) | EE:(1.23,2.45,0.67) | |Vee|:0.34
```

- **Ang**: Current joint angles [θ₁, θ₂, θ₃, θ₄, θ₅]
- **EE**: End-effector position [x, y, z]
- **|Vee|**: End-effector velocity magnitude (for default joint velocities)

---

## 9. Technical Details

### 9.1 Performance Characteristics

**Computational Load (Per Frame):**

| Operation | Complexity | Call Count | Time (approx) |
|-----------|------------|------------|---------------|
| Forward Kinematics | O(5) | 7 | < 0.1 ms |
| Jacobian | O(25) | 1 | < 0.5 ms |
| Rendering | O(N) | 1 | 1-2 ms |
| **Total** | | | **< 3 ms** |

**Frame Rate:** 60 FPS (16.67 ms/frame) → Sufficient margin

### 9.2 Numerical Stability

**Potential Problems:**

1. **Inverse Kinematics:**
   - Acos domain error: `cos(θ) ∈ [-1, 1]` check
   - Solution: `Clamp(value, -1, 1)`

2. **Jacobian Calculation:**
   - Division by zero: h value should not be too small
   - Solution: h = 0.1° (sufficiently large)

3. **Gimbal Lock:**
   - Consecutive X axis rotations → Potential singularity
   - Solution: Control with joint limits

**Numerical Precision:**
- Float (32-bit): ~7 decimal places
- Angle resolution: 0.01° (sufficient)

### 9.3 Coordinate System Consistency

**Critical**: DrawRobot() and ComputeForwardKinematics() functions must use **exactly** the same transformation sequence.

Otherwise:
- Visual and calculated positions will be inconsistent
- IK solution goes to wrong targets
- Jacobian matrix calculated incorrectly

**Verification Method:**
For any joint configuration:
```
visual_position ≈ FK(θ₁, θ₂, θ₃, θ₄, θ₅)
```

### 9.4 Joint Limits

```csharp
// Physical limits (in degrees)
θ₂ ∈ [-90, +90]    // Shoulder
θ₃ ∈ [-150, +150]  // Elbow
θ₄ ∈ [-90, +90]    // Wrist
```

**Why necessary:**
- Physical robots have mechanical limits
- Prevents self-collision
- Realistic motion range

**Limit Check:**
```csharp
θ₂ = Clamp(θ₂, -90, 90);
```

### 9.5 Singularities

**What is Singularity?**

The robot loses degrees of freedom in certain configurations. The Jacobian matrix determinant becomes zero.

**Potential Singularities in This System:**

1. **Base Singularity:**
   - Target above origin (r = 0)
   - θ₁ becomes undefined
   - Solution: Minimal r threshold

2. **Arm Fully Extended:**
   - d = L₂ + L₃ + L₄
   - θ₃ ≈ 0 (Arm straight)
   - Small motion requires large joint changes

3. **Arm Fully Bent:**
   - θ₃ ≈ ±150°
   - Wrist and elbow overlap

**Singularity Check:**
```
det(J·Jᵀ) < ε  →  Singularity near
```

Where ε is small threshold value (e.g. 0.001).

---

## 10. Mathematical References

### 10.1 Summary of Formulas Used

| Concept | Formula | Section |
|---------|---------|---------|
| Forward Kinematics | **p** = FK(**q**) = T₀⁵ · [0,0,0,1]ᵀ | 3 |
| Inverse Kinematics | θ₁ = atan2(x, z) | 4 |
| Law of Cosines | cos(C) = (a² + b² - c²) / (2ab) | 4 |
| Jacobian | J[i,j] = ∂pᵢ/∂θⱼ | 5 |
| Numerical Derivative | f'(x) ≈ [f(x+h) - f(x)] / h | 5 |
| Velocity Kinematics | **v** = J(**q**) · **q̇** | 5 |
| LERP | q(t) = q₀ + (q₁ - q₀) · t | 6 |
| Spherical Camera | x = d·cos(φ)·cos(θ) | 7 |

### 10.2 Notation Table

| Symbol | Meaning | Unit |
|--------|---------|------|
| θᵢ | ith joint angle | degree or radian |
| **q** | Joint angles vector [θ₁,...,θ₅]ᵀ | - |
| **p** | Cartesian position [x,y,z]ᵀ | unit |
| Lᵢ | ith link length | unit |
| **J** | Jacobian matrix (3×5) | unit/radian |
| **v** | Linear velocity vector | unit/second |
| **q̇** | Joint angular velocities | radian/second |
| T | Homogeneous transformation matrix (4×4) | - |

### 10.3 Coordinate Systems

**World Coordinates (World Frame):**
- Origin: Robot base
- Y axis: Up (anti-gravity)

**Joint Coordinates (Joint Frame):**
- Each joint has its local coordinate system
- Rotation axis: X or Y

**End-Effector Coordinates (End-Effector Frame):**
- At last joint center
- Orientation determined by θ₅

---

## 11. References and Further Reading

### 11.1 Fundamental Robotics Books

1. **Craig, J.J.** (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson Education.
   - Fundamental reference for kinematics and dynamics
   - Detailed explanation of Forward/Inverse Kinematics and Jacobian

2. **Spong, M.W., Hutchinson, S., & Vidyasagar, M.** (2020). *Robot Modeling and Control* (2nd ed.). Wiley.
   - DH parameters and transformations
   - Robot control algorithms

3. **Lynch, K.M., & Park, F.C.** (2017). *Modern Robotics: Mechanics, Planning, and Control*. Cambridge University Press.
   - Modern approaches and screw theory
   - **[Free online: modernrobotics.org](http://modernrobotics.org)**

4. **Siciliano, B., Sciavicco, L., Villani, L., & Oriolo, G.** (2010). *Robotics: Modelling, Planning and Control*. Springer.
   - Advanced control theory
   - Trajectory planning and manipulation

5. **Corke, P.I.** (2017). *Robotics, Vision and Control: Fundamental Algorithms in MATLAB* (2nd ed.). Springer.
   - Practical MATLAB implementations
   - Robotics library examples

### 11.2 Computer Graphics

6. **Shirley, P., & Marschner, S.** (2009). *Fundamentals of Computer Graphics* (3rd ed.). AK Peters/CRC Press.
   - Transformation matrices and render pipeline
   - 3D graphics mathematics

7. **Shreiner, D., et al.** (2013). *OpenGL Programming Guide* (8th ed.). Addison-Wesley.
   - OpenGL API reference (Red Book)
   - Shader programming fundamentals

### 11.3 Mathematical Foundations

8. **Strang, G.** (2016). *Introduction to Linear Algebra* (5th ed.). Wellesley-Cambridge Press.
   - Matrix operations and linear transformations
   - Supported by MIT OpenCourseWare lectures

9. **Press, W.H., et al.** (2007). *Numerical Recipes: The Art of Scientific Computing* (3rd ed.). Cambridge University Press.
   - Numerical derivatives and optimization
   - Practical algorithm implementations

### 11.4 C# and .NET Programming

10. **Albahari, J., & Albahari, B.** (2021). *C# 10 in a Nutshell*. O'Reilly Media.
    - C# language features and .NET framework
    - LINQ and async programming

### 11.5 Online Resources

11. **[OpenTK Documentation](https://opentk.net/)**
    - Complete OpenTK API reference

12. **[Learn OpenGL](https://learnopengl.com/)**
    - Comprehensive modern OpenGL tutorial

13. **[Robot Academy - QUT](https://robotacademy.net.au/)**
    - Peter Corke's video lecture series
    - Free robotics education

14. **[Stanford CS223A - Introduction to Robotics](https://see.stanford.edu/Course/CS223A)**
    - Prof. Oussama Khatib video lectures
    - Detailed explanation of kinematics and dynamics

15. **[IEEE Robotics and Automation Society](https://www.ieee-ras.org/)**
    - Academic papers and conferences
    - Robotics research trends

---

## Appendices

### A. Code Structure

```
RobotArm5DOF/
├── Program.cs              (Main code - 617 lines)
│   ├── Kinematic Variables (θ₁-θ₅, L₁-L₄)
│   ├── OnLoad()            (OpenGL initialization)
│   ├── OnUpdateFrame()     (Physics and input)
│   ├── OnRenderFrame()     (Render loop)
│   ├── DrawRobot()         (Robot drawing)
│   ├── DrawCylinder()      (Geometry)
│   ├── ComputeForwardKinematics()
│   ├── SolveIK()           (Inverse kinematics)
│   ├── ComputeJacobian()   (Jacobian matrix)
│   └── Lerp()              (Trajectory)
├── RobotArm5DOF.csproj
├── packages.config
└── README.md               (This file)
```

### B. Quick Reference Card

**Joint Control:**
```
Q/A: Base (θ₁)      W/S: Shoulder (θ₂)   E/D: Elbow (θ₃)
R/F: Wrist (θ₄)     T/G: Roll (θ₅)       X/Z: Gripper
```

**Camera:**
```
Arrows: Rotate      PgUp/PgDn: Zoom      ESC: Exit
```

**IK:**
```
I: Instant go       Y: Smooth go
```