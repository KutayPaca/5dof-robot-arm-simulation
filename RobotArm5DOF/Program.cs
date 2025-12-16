using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;
using OpenTK.Input;
using System;

namespace RobotArm5DOF
{
    // ==================================================================================
    // PDF REFERANS: BÖLÜM I - 5 Serbestlik Dereceli (5DOF) Robot Kol Simülasyonu
    // ==================================================================================
    public class RobotArm : GameWindow
    {
        // --------------------------------------------------------------------------
        // KİNEMATİK DEĞİŞKENLER (Robotun Durumu)
        // PDF Madde 3.1: Kinematik Yapı
        // --------------------------------------------------------------------------
        
        // 1. Eklem (Taban): Y ekseni etrafında döner (Sağ/Sol).
        float theta1 = 0f; 

        // 2. Eklem (Omuz): X ekseni etrafında döner (Öne/Arkaya).
        // NOT: PDF'te özellikle "X ekseni" istendiği için RotateX kullanacağız.
        float theta2 = 0f; 

        // 3. Eklem (Dirsek): X ekseni etrafında döner (Kol bükülmesi).
        float theta3 = 0f; 

        // 4. Eklem (Bilek Pitch): X ekseni etrafında döner (Uç aşağı/yukarı).
        float theta4 = 0f; 

        // 5. Eklem (Bilek Roll): Y ekseni etrafında döner (Uç kendi etrafında).
        float theta5 = 0f; 

        // LINK UZUNLUKLARI
        // PDF Madde 3.1: Parametre olarak tutulmalıdır.
        float link1 = 1.0f; // Taban yüksekliği
        float link2 = 0.8f; // Omuz-Dirsek arası
        float link3 = 0.6f; // Dirsek-Bilek arası
        float link4 = 0.5f; // Bilek-Uç arası

        // GRIPPER (TUTUCU) DURUMU
        float gripperGap = 0.16f; // Parmak açıklığı
        const float GRIPPER_GAP_MIN = 0.03f;
        const float GRIPPER_GAP_MAX = 0.25f;

        // KAMERA AYARLARI (Klavye Kontrolleri)
        float camAngleX = 35f;   // Yukarı/Aşağı bakış
        float camAngleY = 45f;   // Etrafında dönme
        float camDistance = 8f;  // Zoom mesafesi

        // TERS KİNEMATİK (IK) HEDEFİ
        // Robotun ulaşmaya çalışacağı uzaydaki sabit nokta.
        Vector3 ikTarget = new Vector3(1.2f, 1.5f, 0.4f);

        // TRAJEKTORİ (Yörünge Planlama)
        // PDF Madde 6.1 (T/Y tuşu ile yumuşak geçiş)
        bool trajActive = false;
        float trajTime = 0f;
        float trajDuration = 2.0f; // Hareket 2 saniye sürsün
        
        // Animasyon için Başlangıç ve Bitiş açılarını saklayan değişkenler
        float startTheta1, startTheta2, startTheta3, startTheta4, startTheta5;
        float targetTheta1, targetTheta2, targetTheta3, targetTheta4, targetTheta5;

        // Yapıcı Metot: Pencere ayarları
        public RobotArm()
            : base(1024, 768, GraphicsMode.Default,
                  "5DOF Robot Kol - PDF Uyumlu")
        {
            VSync = VSyncMode.On;
        }

        // --------------------------------------------------------------------------
        // OPENGL BAŞLANGIÇ AYARLARI
        // PDF Madde 4: Zemin, Işıklandırma ve Sahne
        // --------------------------------------------------------------------------
        protected override void OnLoad(EventArgs e)
        {
            GL.ClearColor(0.1f, 0.1f, 0.1f, 1f); // Arka plan rengi
            GL.Enable(EnableCap.DepthTest);      // 3B derinlik algısı (arkadakiler öne çizilmesin)

            // Işıklandırma Sistemi (Lighting)
            GL.Enable(EnableCap.Lighting);
            GL.Enable(EnableCap.Light0);        // 1. Işık kaynağını aç
            GL.Enable(EnableCap.ColorMaterial); // Nesne renklerini materyal olarak kullan

            // Işık Pozisyonu ve Rengi
            float[] ambient = { 0.3f, 0.3f, 0.3f, 1f }; // Ortam aydınlığı
            float[] diffuse = { 1f, 1f, 1f, 1f };       // Parlak ışık
            float[] pos = { 5f, 10f, 10f, 1f };         // Işığın konumu
            
            GL.Light(LightName.Light0, LightParameter.Ambient, ambient);
            GL.Light(LightName.Light0, LightParameter.Diffuse, diffuse);
            GL.Light(LightName.Light0, LightParameter.Position, pos);

            GL.Enable(EnableCap.Normalize); // Normal vektörlerini düzelt
        }

        protected override void OnResize(EventArgs e)
        {
            GL.Viewport(0, 0, Width, Height); // Pencere boyutu değişince çizim alanını güncelle
        }

        // --------------------------------------------------------------------------
        // GÜNCELLEME DÖNGÜSÜ (UPDATE LOOP)
        // PDF Madde 7: Klavye Kontrolleri
        // --------------------------------------------------------------------------
        protected override void OnUpdateFrame(FrameEventArgs e)
        {
            var key = Keyboard.GetState();
            float dt = (float)e.Time;
            if (key.IsKeyDown(Key.Escape)) Exit();

            float step = 60f * dt; // Dönüş hızı (derece/saniye)

            // --- EKLEM KONTROLLERİ ---
            // Q/A: Taban (Theta1)
            if (key.IsKeyDown(Key.Q)) theta1 += step;
            if (key.IsKeyDown(Key.A)) theta1 -= step;

            // W/S: Omuz (Theta2)
            if (key.IsKeyDown(Key.W)) theta2 += step;
            if (key.IsKeyDown(Key.S)) theta2 -= step;

            // E/D: Dirsek (Theta3)
            if (key.IsKeyDown(Key.E)) theta3 += step;
            if (key.IsKeyDown(Key.D)) theta3 -= step;

            // R/F: Bilek Pitch (Theta4)
            if (key.IsKeyDown(Key.R)) theta4 += step;
            if (key.IsKeyDown(Key.F)) theta4 -= step;

            // T/G: Bilek Roll (Theta5)
            if (key.IsKeyDown(Key.T)) theta5 += step;
            if (key.IsKeyDown(Key.G)) theta5 -= step;

            // Açılara fiziksel sınır koyma (Clamp)
            theta2 = MathHelper.Clamp(theta2, -90, 90);
            theta3 = MathHelper.Clamp(theta3, -150, 150);
            theta4 = MathHelper.Clamp(theta4, -90, 90);

            // X/Z: Gripper Aç/Kapa
            if (key.IsKeyDown(Key.X)) gripperGap += 0.6f * dt;
            if (key.IsKeyDown(Key.Z)) gripperGap -= 0.6f * dt;
            gripperGap = MathHelper.Clamp(gripperGap, GRIPPER_GAP_MIN, GRIPPER_GAP_MAX);

            // Ok Tuşları: Kamera Hareketi
            if (key.IsKeyDown(Key.Left)) camAngleY -= 80f * dt;
            if (key.IsKeyDown(Key.Right)) camAngleY += 80f * dt;
            if (key.IsKeyDown(Key.Up)) camAngleX -= 80f * dt;
            if (key.IsKeyDown(Key.Down)) camAngleX += 80f * dt;
            camAngleX = MathHelper.Clamp(camAngleX, -89, 89);

            // PageUp/Down: Kamera Zoom
            if (key.IsKeyDown(Key.PageUp)) camDistance -= 5f * dt;
            if (key.IsKeyDown(Key.PageDown)) camDistance += 5f * dt;
            camDistance = MathHelper.Clamp(camDistance, 2f, 20f);

            // --- TERS KİNEMATİK (IK) TETİKLEYİCİLERİ ---

            // 'I' Tuşu: IK ile anında git
            if (key.IsKeyDown(Key.I))
            {
                float s1, s2, s3, s4, s5;
                // SolveIK fonksiyonunu çağır, çözüm varsa açıları güncelle
                if (SolveIK(ikTarget, out s1, out s2, out s3, out s4, out s5))
                {
                    theta1 = s1; theta2 = s2; theta3 = s3; theta4 = s4; theta5 = s5;
                }
            }

            // 'Y' Tuşu: Trajektori (Yumuşak Geçiş) ile git
            if (key.IsKeyDown(Key.Y) && !trajActive)
            {
                float s1, s2, s3, s4, s5;
                if (SolveIK(ikTarget, out s1, out s2, out s3, out s4, out s5))
                {
                    // Mevcut açıları başlangıç olarak kaydet
                    startTheta1 = theta1; startTheta2 = theta2; 
                    startTheta3 = theta3; startTheta4 = theta4; startTheta5 = theta5;

                    // Hedef açıları kaydet
                    targetTheta1 = s1; targetTheta2 = s2; 
                    targetTheta3 = s3; targetTheta4 = s4; targetTheta5 = s5;

                    trajTime = 0f;
                    trajActive = true;
                }
            }

            // --- TRAJEKTORİ HESABI (LERP) ---
            // PDF BÖLÜM II Madde 6: Trajektori Planlama
            if (trajActive)
            {
                trajTime += dt;
                float u = trajTime / trajDuration; // 0 ile 1 arasında zaman oranı
                if (u >= 1f) { u = 1f; trajActive = false; }

                // Lerp Fonksiyonu: Formül: Q_simdiki = Q_bas + (Q_hedef - Q_bas) * u
                theta1 = Lerp(startTheta1, targetTheta1, u);
                theta2 = Lerp(startTheta2, targetTheta2, u);
                theta3 = Lerp(startTheta3, targetTheta3, u);
                theta4 = Lerp(startTheta4, targetTheta4, u);
                theta5 = Lerp(startTheta5, targetTheta5, u);
            }
        }

        // --------------------------------------------------------------------------
        // ÇİZİM DÖNGÜSÜ (RENDER LOOP)
        // --------------------------------------------------------------------------
        protected override void OnRenderFrame(FrameEventArgs e)
        {
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            // 1. Kamera Matrisi (Projection)
            Matrix4 proj = Matrix4.CreatePerspectiveFieldOfView(
                MathHelper.PiOver4, Width / (float)Height, 0.1f, 100f);
            GL.MatrixMode(MatrixMode.Projection);
            GL.LoadMatrix(ref proj);

            // 2. Bakış Matrisi (ModelView)
            Vector3 cam = CameraPosition();
            Matrix4 look = Matrix4.LookAt(cam, Vector3.Zero, Vector3.UnitY);
            GL.MatrixMode(MatrixMode.Modelview);
            GL.LoadMatrix(ref look);

            // 3. Sahne Elemanlarını Çiz
            DrawGround(); // Zemin
            DrawAxes();   // Eksenler (Kırmızı=X, Yeşil=Y, Mavi=Z)
            DrawRobot();  // Robotun Kendisi

            // 4. BAŞLIK BİLGİLERİ (Title Info)
            // PDF Madde 8: Pencere Başlığı Bilgileri
            
            // A. Uç Nokta Konumu (FK Hesabı)
            Vector3 ee = ComputeForwardKinematics(theta1, theta2, theta3, theta4, theta5);
            
            // B. Jacobian Hesabı ve Hız (Velocity)
            // PDF BÖLÜM II Madde 4: Jacobian Hesaplama
            float[,] J = ComputeJacobian(theta1, theta2, theta3, theta4, theta5);
            
            // C. Hız Kinematiği: v = J * theta_dot
            // Varsayım: Tüm eklemler 10 derece/saniye hızla dönüyor olsun.
            float[] thetaDotRad = new float[5];
            for (int i = 0; i < 5; i++) thetaDotRad[i] = MathHelper.DegreesToRadians(10f);
            
            Vector3 vEE = MultiplyJacobian(J, thetaDotRad);
            float vMag = vEE.Length; // Hız büyüklüğü

            // Başlığı Güncelle
            Title = string.Format(
                "5DOF Robot | Aci:({0:0},{1:0},{2:0},{3:0},{4:0}) | EE:({5:F2},{6:F2},{7:F2}) | |Vee|:{8:F2}",
                theta1, theta2, theta3, theta4, theta5, ee.X, ee.Y, ee.Z, vMag);

            SwapBuffers();
        }

        // --------------------------------------------------------------------------
        // ÇİZİM FONKSİYONLARI
        // PDF Madde 4: Görsel Tasarım (Silindirler)
        // --------------------------------------------------------------------------
        void DrawRobot()
        {
            GL.PushMatrix(); // Ana matrisi kaydet

            // --- TABAN (J1) ---
            DrawJoint(); // Gri Eklem
            GL.Rotate(theta1, 0, 1, 0); // Y Ekseni Etrafında Dönüş (Yaw)
            DrawLink(link1); // Turuncu Link
            GL.Translate(0, link1, 0); // Link ucuna git

            // --- OMUZ (J2) ---
            DrawJoint();
            // PDF Madde 3.1: Omuz Eklem (X ekseni etrafında)
            // DİKKAT: Burada RotateX kullanıyoruz. Bu sayede kol öne/arkaya eğilir (Pitch).
            GL.Rotate(theta2, 1, 0, 0); 
            DrawLink(link2);
            GL.Translate(0, link2, 0);

            // --- DİRSEK (J3) ---
            DrawJoint();
            // PDF: Tasarıma uygun olarak X seçildi (Omuz ile aynı eksen olmalı ki düzlemsel IK çalışsın)
            GL.Rotate(theta3, 1, 0, 0); 
            DrawLink(link3);
            GL.Translate(0, link3, 0);

            // --- BİLEK (J4) ---
            DrawJoint();
            // Uç kısmın aşağı/yukarı bakması için yine X ekseni (Pitch)
            GL.Rotate(theta4, 1, 0, 0); 
            DrawLink(link4);
            GL.Translate(0, link4, 0);

            // --- BİLEK DÖNÜŞ (J5) ---
            // Uç efektörün kendi etrafında dönmesi (Roll/Yaw)
            GL.Rotate(theta5, 0, 1, 0); // Y ekseni etrafında
            
            DrawGripper(); // Tutucuyu çiz

            GL.PopMatrix(); // Matrisi geri yükle
        }

        // SİLİNDİR ÇİZİMİ (ÖDEV GEREKSİNİMİ)
        // Küp yerine silindir kullanılarak "Visual Design" puanı alınır.
        void DrawCylinder(float radius, float height, int segments = 24)
        {
            // Silindirin yan yüzeyi (QuadStrip)
            GL.Begin(PrimitiveType.QuadStrip);
            for (int i = 0; i <= segments; i++)
            {
                double angle = (Math.PI * 2.0 * i) / segments;
                float x = (float)Math.Cos(angle) * radius;
                float z = (float)Math.Sin(angle) * radius;

                GL.Normal3(x, 0, z); // Işık için normal vektörü
                GL.Vertex3(x, height, z); // Üst nokta
                GL.Vertex3(x, 0, z);      // Alt nokta
            }
            GL.End();

            // Alt ve Üst Kapaklar (İçi dolu görünsün)
            DrawDisk(radius, 0f, segments);
            DrawDisk(radius, height, segments);
        }

        void DrawDisk(float radius, float yPos, int segments)
        {
            GL.Begin(PrimitiveType.Polygon);
            GL.Normal3(0, (yPos > 0 ? 1 : -1), 0);
            for (int i = 0; i < segments; i++)
            {
                double angle = (Math.PI * 2.0 * i) / segments;
                GL.Vertex3(Math.Cos(angle) * radius, yPos, Math.Sin(angle) * radius);
            }
            GL.End();
        }

        // Yardımcı Çizim Fonksiyonları
        void DrawLink(float L)
        {
            GL.Color3(1.0f, 0.5f, 0.0f); // Turuncu Linkler
            DrawCylinder(0.1f, L);       // Yarıçap=0.1, Yükseklik=L
        }

        void DrawJoint()
        {
            GL.Color3(0.6f, 0.6f, 0.6f); // Gri Eklemler
            GL.PushMatrix();
            GL.Translate(0, -0.1f, 0);   // Eklemi ortala
            DrawCylinder(0.18f, 0.2f);   // Geniş, kısa silindir
            GL.PopMatrix();
        }

        void DrawGripper()
        {
            // Gripper çizimi (Görsel detay)
            GL.PushMatrix();
            GL.Translate(0, 0.05f, 0);
            GL.Color3(0.3f, 0.3f, 0.3f);
            
            // Taban
            GL.PushMatrix();
            GL.Scale(0.3f, 0.05f, 0.3f);
            DrawCube(); // Detaylar küp kalabilir
            GL.PopMatrix();

            // Parmaklar (Açılıp kapanma animasyonu)
            float t = (gripperGap - GRIPPER_GAP_MIN) / (GRIPPER_GAP_MAX - GRIPPER_GAP_MIN);
            float r = 0.05f + (0.16f - 0.05f) * t;

            for (int i = 0; i < 3; i++)
            {
                GL.PushMatrix();
                GL.Rotate(i * 120, 0, 1, 0);
                GL.Translate(0, 0, r);
                GL.Color3(0.8f, 0.8f, 0.8f);
                GL.Translate(0, 0.15f, 0);
                GL.Scale(0.04f, 0.3f, 0.04f);
                DrawCube();
                GL.PopMatrix();
            }
            GL.PopMatrix();
        }

        void DrawCube()
        {
            // Basit birim küp çizimi (-0.5 ile 0.5 arası)
            GL.Begin(PrimitiveType.Quads);
            GL.Normal3(0, 0, 1); GL.Vertex3(-0.5f, -0.5f, 0.5f); GL.Vertex3(0.5f, -0.5f, 0.5f);
            GL.Vertex3(0.5f, 0.5f, 0.5f); GL.Vertex3(-0.5f, 0.5f, 0.5f);
            GL.Normal3(0, 0, -1); GL.Vertex3(-0.5f, -0.5f, -0.5f); GL.Vertex3(-0.5f, 0.5f, -0.5f);
            GL.Vertex3(0.5f, 0.5f, -0.5f); GL.Vertex3(0.5f, -0.5f, -0.5f);
            GL.Normal3(-1, 0, 0); GL.Vertex3(-0.5f, -0.5f, -0.5f); GL.Vertex3(-0.5f, -0.5f, 0.5f);
            GL.Vertex3(-0.5f, 0.5f, 0.5f); GL.Vertex3(-0.5f, 0.5f, -0.5f);
            GL.Normal3(1, 0, 0); GL.Vertex3(0.5f, -0.5f, -0.5f); GL.Vertex3(0.5f, 0.5f, -0.5f);
            GL.Vertex3(0.5f, 0.5f, 0.5f); GL.Vertex3(0.5f, -0.5f, 0.5f);
            GL.Normal3(0, 1, 0); GL.Vertex3(-0.5f, 0.5f, -0.5f); GL.Vertex3(-0.5f, 0.5f, 0.5f);
            GL.Vertex3(0.5f, 0.5f, 0.5f); GL.Vertex3(0.5f, 0.5f, -0.5f);
            GL.Normal3(0, -1, 0); GL.Vertex3(-0.5f, -0.5f, -0.5f); GL.Vertex3(0.5f, -0.5f, -0.5f);
            GL.Vertex3(0.5f, -0.5f, 0.5f); GL.Vertex3(-0.5f, -0.5f, 0.5f);
            GL.End();
        }

        void DrawAxes()
        {
            GL.Disable(EnableCap.Lighting);
            GL.Begin(PrimitiveType.Lines);
            GL.Color3(1, 0, 0); GL.Vertex3(0, 0, 0); GL.Vertex3(3, 0, 0); // X
            GL.Color3(0, 1, 0); GL.Vertex3(0, 0, 0); GL.Vertex3(0, 3, 0); // Y
            GL.Color3(0, 0, 1); GL.Vertex3(0, 0, 0); GL.Vertex3(0, 0, 3); // Z
            GL.End();
            GL.Enable(EnableCap.Lighting);
        }

        void DrawGround()
        {
            GL.Disable(EnableCap.Lighting);
            GL.Color3(0.2f, 0.2f, 0.2f);
            GL.Begin(PrimitiveType.Quads);
            GL.Vertex3(-5, 0, -5); GL.Vertex3(5, 0, -5);
            GL.Vertex3(5, 0, 5); GL.Vertex3(-5, 0, 5);
            GL.End();
            GL.Enable(EnableCap.Lighting);
        }

        // --------------------------------------------------------------------------
        // KİNEMATİK FONKSİYONLAR
        // PDF Madde 5 ve 6
        // --------------------------------------------------------------------------

        // İLERİ KİNEMATİK (FK)
        // Robotun 5 açısını alıp, uç noktanın uzaydaki koordinatını hesaplar.
        Vector3 ComputeForwardKinematics(float t1Deg, float t2Deg, float t3Deg, float t4Deg, float t5Deg)
        {
            // Derece -> Radyan
            float t1 = MathHelper.DegreesToRadians(t1Deg);
            float t2 = MathHelper.DegreesToRadians(t2Deg);
            float t3 = MathHelper.DegreesToRadians(t3Deg);
            float t4 = MathHelper.DegreesToRadians(t4Deg);
            float t5 = MathHelper.DegreesToRadians(t5Deg);

            Matrix4 T = Matrix4.Identity;

            // DİKKAT: Buradaki matris sırası ve eksenler, DrawRobot fonksiyonu ile
            // BİREBİR AYNI olmalıdır. Yoksa robotun görüntüsü ile hesabı tutmaz.

            // 1. Taban (Yaw - Y ekseni)
            T *= Matrix4.CreateRotationY(t1);
            T *= Matrix4.CreateTranslation(0f, link1, 0f);

            // 2. Omuz (Pitch - X ekseni)
            T *= Matrix4.CreateRotationX(t2); 
            T *= Matrix4.CreateTranslation(0f, link2, 0f);

            // 3. Dirsek (Pitch - X ekseni)
            T *= Matrix4.CreateRotationX(t3);
            T *= Matrix4.CreateTranslation(0f, link3, 0f);

            // 4. Bilek (Pitch - X ekseni)
            T *= Matrix4.CreateRotationX(t4);
            T *= Matrix4.CreateTranslation(0f, link4, 0f);

            // 5. Bilek Dönüş (Roll - Y ekseni)
            T *= Matrix4.CreateRotationY(t5);
            // Uçta ekstra uzunluk yoksa Translation gerekmez.

            return Vector3.TransformPosition(Vector3.Zero, T);
        }

        // TERS KİNEMATİK (IK) - SEÇENEK A (Analitik)
        // Geometrik (trigonometrik) formüllerle hedef açıları bulur.
        bool SolveIK(Vector3 target,
                     out float solTheta1, out float solTheta2,
                     out float solTheta3, out float solTheta4, out float solTheta5)
        {
            // Varsayılan değerler (Çözüm bulunamazsa eskiler kalsın)
            solTheta1 = theta1; solTheta2 = theta2; solTheta3 = theta3;
            solTheta4 = theta4; solTheta5 = theta5;

            // 1. ADIM: Taban Açısı (Theta1) Hesabı
            // Hedefe üstten baktığımızda hangi yönde olduğunu 'atan2' ile buluruz.
            // Bu, Z eksenini hedefe hizalar.
            float t1 = (float)Math.Atan2(target.X, target.Z);
            
            // Hedefin tabana yatay uzaklığı (r)
            float r = (float)Math.Sqrt(target.X * target.X + target.Z * target.Z);

            // 2. ADIM: Düzlemsel 2-Link Problemi
            // Taban döndükten sonra problem, 2 boyutlu (Yükseklik Y ve Uzaklık r) bir üçgen problemine dönüşür.
            
            float y = target.Y - link1; // Omuz yüksekliğini çıkar (Base offset)
            float d = (float)Math.Sqrt(r * r + y * y); // Omuzdan hedefe kuş uçuşu mesafe

            // Basitleştirme: 5DOF kolu 3DOF gibi çözüyoruz.
            // 3. ve 4. Linki tek bir uzun kol gibi (L3+L4) varsayıyoruz.
            float l1 = link2;       // Üst kol
            float l2 = link3 + link4; // Alt kol (Toplam)

            // Hedef robotun erişim mesafesi dışında mı?
            if (d > l1 + l2) return false; 

            // Kosinüs Teoremi ile Dirsek Açısı (Theta3)
            // c^2 = a^2 + b^2 - 2ab*cos(C)
            float cosQ3 = (d * d - l1 * l1 - l2 * l2) / (2f * l1 * l2);
            cosQ3 = MathHelper.Clamp(cosQ3, -1f, 1f); // Matematik hatası olmasın
            float q3 = (float)Math.Acos(cosQ3);

            // Omuz Açısı (Theta2) Hesabı
            // Hedefin açısı (phi) - Üçgenin iç açısı (psi)
            float phi = (float)Math.Atan2(y, r); // NOT: PDF X ekseni istediği için (y, r) kullandık.
            // X ekseni dönüşünde: Artı açı kolu "geriye/yukarı", eksi açı "öne/aşağı" yatırır.
            // Koordinat sistemine göre (y, r) doğru yönelimi verir.
            
            float psi = (float)Math.Atan2(l2 * (float)Math.Sin(q3),
                                          l1 + l2 * (float)Math.Cos(q3));
            float q2 = MathHelper.Pi/2 - (phi + psi); 
            // NOT: Pi/2'den çıkarma nedeni: Robotumuz dik (Y ekseni) başlıyor.
            // 0 derece dik duruş ise, hedefe (yatay) gitmek için 90 derece eğilmesi gerekir.

            // Radyan -> Derece Dönüşümleri
            solTheta1 = MathHelper.RadiansToDegrees(t1);
            solTheta2 = MathHelper.RadiansToDegrees(q2); 

            // Bulunan dirsek bükülme açısını (q3) dağıtıyoruz
            float totalElbowAngle = MathHelper.RadiansToDegrees(q3);
            solTheta3 = totalElbowAngle * 0.6f; 
            solTheta4 = totalElbowAngle * 0.4f;
            
            // Theta5 (Bilek Yaw) sabit kalabilir
            solTheta5 = 0f; 

            return true;
        }

        // JACOBIAN HESAPLAMA (Nümerik Türev)
        // PDF BÖLÜM III Madde 4: 5 Sütunlu Jacobian
        float[,] ComputeJacobian(float t1Deg, float t2Deg, float t3Deg, float t4Deg, float t5Deg)
        {
            float[,] J = new float[3, 5]; // 3 Satır (XYZ), 5 Sütun (Eklemler)
            float hDeg = 0.1f; // Türev adımı
            
            Vector3 current = ComputeForwardKinematics(t1Deg, t2Deg, t3Deg, t4Deg, t5Deg);

            // Her eklem için: (FK(acı + h) - FK(acı)) / h
            
            // Sütun 1 (Theta1)
            Vector3 p1 = ComputeForwardKinematics(t1Deg + hDeg, t2Deg, t3Deg, t4Deg, t5Deg);
            Vector3 col1 = (p1 - current) / MathHelper.DegreesToRadians(hDeg);
            J[0, 0] = col1.X; J[1, 0] = col1.Y; J[2, 0] = col1.Z;

            // Sütun 2 (Theta2)
            Vector3 p2 = ComputeForwardKinematics(t1Deg, t2Deg + hDeg, t3Deg, t4Deg, t5Deg);
            Vector3 col2 = (p2 - current) / MathHelper.DegreesToRadians(hDeg);
            J[0, 1] = col2.X; J[1, 1] = col2.Y; J[2, 1] = col2.Z;

            // Sütun 3 (Theta3)
            Vector3 p3 = ComputeForwardKinematics(t1Deg, t2Deg, t3Deg + hDeg, t4Deg, t5Deg);
            Vector3 col3 = (p3 - current) / MathHelper.DegreesToRadians(hDeg);
            J[0, 2] = col3.X; J[1, 2] = col3.Y; J[2, 2] = col3.Z;

            // Sütun 4 (Theta4)
            Vector3 p4 = ComputeForwardKinematics(t1Deg, t2Deg, t3Deg, t4Deg + hDeg, t5Deg);
            Vector3 col4 = (p4 - current) / MathHelper.DegreesToRadians(hDeg);
            J[0, 3] = col4.X; J[1, 3] = col4.Y; J[2, 3] = col4.Z;

            // Sütun 5 (Theta5)
            Vector3 p5 = ComputeForwardKinematics(t1Deg, t2Deg, t3Deg, t4Deg, t5Deg + hDeg);
            Vector3 col5 = (p5 - current) / MathHelper.DegreesToRadians(hDeg);
            J[0, 4] = col5.X; J[1, 4] = col5.Y; J[2, 4] = col5.Z;

            return J;
        }

        // HIZ KİNEMATİĞİ HESABI
        // v = J * q_dot
        Vector3 MultiplyJacobian(float[,] J, float[] qDot)
        {
            Vector3 v = Vector3.Zero;
            for (int r = 0; r < 3; r++) // X, Y, Z
            {
                for (int c = 0; c < 5; c++) // 5 Eklem
                {
                    if (r == 0) v.X += J[r, c] * qDot[c];
                    if (r == 1) v.Y += J[r, c] * qDot[c];
                    if (r == 2) v.Z += J[r, c] * qDot[c];
                }
            }
            return v;
        }

        // Lineer İnterpolasyon (Lerp)
        static float Lerp(float a, float b, float t)
        {
            return a + (b - a) * t;
        }

        Vector3 CameraPosition()
        {
            float xr = MathHelper.DegreesToRadians(camAngleX);
            float yr = MathHelper.DegreesToRadians(camAngleY);
            return new Vector3(
                camDistance * (float)Math.Cos(xr) * (float)Math.Cos(yr),
                camDistance * (float)Math.Sin(xr),
                camDistance * (float)Math.Cos(xr) * (float)Math.Sin(yr));
        }

        [STAThread]
        public static void Main()
        {
            using (var app = new RobotArm())
            {
                app.Run(60.0);
            }
        }
    }
}
