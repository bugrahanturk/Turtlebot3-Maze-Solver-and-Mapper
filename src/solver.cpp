#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <cmath>

class TurtleBot
{
    public:
        double on_uzaklik            ; // 15-345  (derece)
        double sol_uzaklik           ; // 75-105  (derece)
        double sag_uzaklik           ; // 255-285 (derece)
        double arka_uzaklik          ; // 165-195 (derece)
        double sag_on_kose_uzaklik   ; // 300-330 (derece)
        double sol_arka_kose_uzaklik ; // 115-125 (derece)
        double sol_on_kose_uzaklik   ; // 30-60   (derece)
    
    public:
        void Uzakliklari_Guncelle(double on, 
                                  double sol,
                                  double sag,
                                  double arka,
                                  double sag_on_kose,
                                  double sol_arka_kose,
                                  double sol_on_kose)
        {
            on_uzaklik = on;
            sol_uzaklik = sol;
            sag_uzaklik = sag;
            arka_uzaklik = arka;
            sag_on_kose_uzaklik = sag_on_kose;
            sol_arka_kose_uzaklik = sol_arka_kose;
            sol_on_kose_uzaklik = sol_on_kose;
        }
};


class MazeSolver {
public:
    MazeSolver() : init_(false), donuste_(false)
    {
        // ROS aboneliklerini tanimlayalim
        scan_sub_ = nh_.subscribe("/scan", 10, &MazeSolver::scanCallback ,this);
        odom_sub_ = nh_.subscribe("/odom", 10, &MazeSolver::odomCallback, this);

        // Hız komutlarını yayınlamak icin publisher olusturalim
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // Duvara ideal mesafe ve paralellik torelansi degerlerini ayarlayalim
        duvara_olan_uzaklik_ = 0.5;
        parallel_band_       = 0.1;

        // Hedefe ulasildi mi kontrolu icin bayrak
        hedefe_ulasildi_mi_ = false;

        // Baslangic bilgisi
        ROS_INFO("MazeSolver baslatildi! Duvara uzaklik Mesafe Miktari : %f, parallel_band uzakligi: %f", duvara_olan_uzaklik_, parallel_band_);
    }

    void spin() 
    {
        ros::Rate rate(10);

        // Hedefe ulasilana kadar donguyu devam ettirelim
        while (ros::ok() && !hedefe_ulasildi_mi_) 
        {
            ros::spinOnce();
            rate.sleep();
        }

        // Robotu durdur
        Robotu_Durdur();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_, odom_sub_;
    ros::Publisher cmd_pub_;
    double duvara_olan_uzaklik_, parallel_band_;
    bool hedefe_ulasildi_mi_,donuste_,init_;
    geometry_msgs::Twist cmd_vel_;
    TurtleBot robot;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) 
    {
        robot.Uzakliklari_Guncelle(Ortalama_Mesafe_Hesapla(scan, 345, 15),
                                   Ortalama_Mesafe_Hesapla(scan, 75, 105),
                                   Ortalama_Mesafe_Hesapla(scan, 255, 285),
                                   Ortalama_Mesafe_Hesapla(scan, 165, 195),
                                   Ortalama_Mesafe_Hesapla(scan, 300, 330),
                                   Ortalama_Mesafe_Hesapla(scan, 115,125),
                                   Ortalama_Mesafe_Hesapla(scan, 30, 60));
       
        // Mesafe bilgilerini  yazdiralim
        ROS_INFO("\nOn: %f\n Sol: %f\n Sag: %f\n Arka: %f\n Sag On Kose: %f\n Sol Arka Kose: %f\n Sol On Kose %f\n", 
                robot.on_uzaklik,
                robot.sol_uzaklik,
                robot.sag_uzaklik,
                robot.arka_uzaklik,
                robot.sag_on_kose_uzaklik,
                robot.sol_arka_kose_uzaklik,
                robot.sol_on_kose_uzaklik);

        // Donuyor mu kontrol edelim
        if (donuste_)
        {
            return;
        } 

        // Eger robot duvara cok yakinsa geri cekilme fonksiyonunu cagiriyoruz
        if (robot.on_uzaklik < 0.15) 
        {
            ROS_WARN("Duvara cok yaklasildi. Geri cekilme baslatiliyor.");
            Geri_Cekilme();
            return;
        }
        
        if (!init_) 
        {
            init_ = true;
            ROS_INFO("Ilk hizalama yapiliyor!!!");
            En_Yakin_Duvara_Soldan_Paralel_Ol(scan);
        } 
        else 
        {
            // Eger bir kose durumu varsa 
            if (Kose_Var_Mi(robot)) 
            {
                ROS_INFO("---KOSE ALGILANDI---");
                Koseyi_Don(robot);
            } 
            else 
            {
                // Paralellik kontrolunu sagla ve duvari takip et
                Paralel_Mesafede_Duz_Ilerle(robot);
            }
            cmd_pub_.publish(cmd_vel_);
        }
    }

    void Koseyi_Don(TurtleBot robot) 
    {
        double esik_min_uzaklik  = duvara_olan_uzaklik_ - parallel_band_;
        double esik_maks_uzaklik = duvara_olan_uzaklik_ + parallel_band_;

        if (robot.on_uzaklik < esik_min_uzaklik &&
            robot.sag_uzaklik > esik_min_uzaklik &&
            robot.sol_uzaklik < esik_min_uzaklik &&
            robot.sol_on_kose_uzaklik < esik_maks_uzaklik) 
        {
            ROS_INFO("90 Derece Kose Algilandi - saga donuluyor");
            Istenilen_Derece_Miktarinca_Don(true,90);
        }
        else if (robot.on_uzaklik  < esik_min_uzaklik &&
                 robot.sag_uzaklik > esik_maks_uzaklik &&
                 robot.sol_uzaklik > esik_maks_uzaklik &&
                 robot.sol_on_kose_uzaklik < esik_min_uzaklik &&
                 robot.sag_on_kose_uzaklik < esik_min_uzaklik)
        {
            ROS_INFO("SAGIM SOLUM BOS ONUM DOLU - SAGA DONEREK SOLA PARALEL OL");
            Istenilen_Derece_Miktarinca_Don(true,90);
        }
        else if (robot.on_uzaklik > esik_maks_uzaklik            && 
                 robot.sag_uzaklik > esik_maks_uzaklik           &&
                 robot.sol_uzaklik < esik_maks_uzaklik           &&
                 robot.sol_on_kose_uzaklik > esik_maks_uzaklik   && 
                 robot.sag_on_kose_uzaklik > esik_maks_uzaklik   &&
                 robot.sol_arka_kose_uzaklik < esik_maks_uzaklik &&
                 robot.arka_uzaklik > esik_maks_uzaklik ) 
        {
            ROS_INFO("270 Derece Kose Algilandi - Acik olan duvara paralel hale gelene kadar donmeye baslaniliyor");
            IKi_Yuz_Yetmis_Derece_Kose_Donusu_Yap(robot);
        }
        else
        {
            // Yanlis kose algilamada duz ilerlemeye devam edelim ki robot durmasin
            Paralel_Mesafede_Duz_Ilerle(robot);
        }
    }

    // 270-Degree Corner Handling Function
    void IKi_Yuz_Yetmis_Derece_Kose_Donusu_Yap(TurtleBot robot)
    {
        // sol acik olana kadar ilerle ve sonra 90 derece sola dondurelim
        cmd_vel_.linear.x  = 0.2;
        cmd_vel_.angular.z = 0.0;
        cmd_pub_.publish(cmd_vel_);
        ros::Rate rate(10);

        // sol taraf acik olana kadar ilerle
        while (true) 
        {
            ROS_INFO("270-derece kose acik olan duvara paralel hale geliniyor.");
            ros::spinOnce();

            if (robot.sol_uzaklik > duvara_olan_uzaklik_ + parallel_band_ || robot.on_uzaklik < 0.15) 
            {
                break;
            }
            
            cmd_pub_.publish(cmd_vel_);
            rate.sleep();
        }
        // Sola 90 derece don
        Istenilen_Derece_Miktarinca_Don(false, 90);

        ROS_INFO("270-derece kose acik olan duvara paralel hale gelindi.");
    }

    void Istenilen_Derece_Miktarinca_Don(bool saga_donus_mu, double derece) 
    {
        donuste_ = true;
        // Ileri hız yok, Sag icin negatif, sol icin pozitif
        cmd_vel_.linear.x = 0.0; 
        cmd_vel_.angular.z = saga_donus_mu ? -0.5 : 0.5; 

        double turn_duration = (derece * M_PI / 180) / std::abs(cmd_vel_.angular.z);
        
        // Komutlari belirli bir sure boyunca yayinlayarak donme islemimi yapalim
        ROS_INFO("Donme Suresi: %f",turn_duration);
        ros::Rate rate(10);
        ros::Time start_time = ros::Time::now();
        while (ros::Time::now() - start_time < ros::Duration(turn_duration)) 
        {
            ROS_INFO("Arac DONUYOR!!!");
            ros::spinOnce();
            cmd_pub_.publish(cmd_vel_);
            rate.sleep();
        }

        // Donus bittiginde bayragi kaldiralim 
        donuste_ = false;
        cmd_vel_.angular.z = 0.0;
        cmd_pub_.publish(cmd_vel_);
        ROS_INFO("90 derece donme tamamlandi.");
    }

    // Robot duvara çok yaklastiginda geri cekilme fonksiyonu
    void Geri_Cekilme() 
    {
        geometry_msgs::Twist cmd_vel;

        cmd_vel.linear.x = -0.2;

        cmd_pub_.publish(cmd_vel);
        // 1 saniye boyunca geri cekilme süresi
        ros::Duration(1.0).sleep(); 

        // Durdur
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_pub_.publish(cmd_vel);
    }

    // Belirli bir aci araligindaki mesafenin ortalamasini hesapla
    double Ortalama_Mesafe_Hesapla(const sensor_msgs::LaserScan::ConstPtr& scan, int baslangic_acisi, int bitis_acisi) 
    {
        int tarama_boyutu = scan->ranges.size();
        double toplam_uzaklik = 0.0;
        int sayac = 0;

        for (int i = baslangic_acisi; i != bitis_acisi; i = (i + 1) % tarama_boyutu) 
        {
            double uzaklik = scan->ranges[i];

            if (uzaklik > 0.1 && uzaklik < scan->range_max) 
            {
                toplam_uzaklik += uzaklik;
                sayac++;
            }
        }

        if (sayac > 0) 
        {
            return toplam_uzaklik / sayac;
        } 
        else
         {
            return std::numeric_limits<double>::infinity();
        }
    }

    // En yakin duvarin acisini ve mesafesini bulmaya yarar
    double En_Yakin_Duvarin_Acisini_Bul(const sensor_msgs::LaserScan::ConstPtr& scan) 
    {
        double minimum_uzaklik = std::numeric_limits<double>::infinity();
        int min_index = -1;

        for (int i = 0; i < scan->ranges.size(); ++i) 
        {
            double uzaklik = scan->ranges[i];

            if (uzaklik < minimum_uzaklik) 
            {
                minimum_uzaklik = uzaklik;
                min_index = i;
            }
        }
        return min_index;
    }

    void En_Yakin_Duvara_Soldan_Paralel_Ol(const sensor_msgs::LaserScan::ConstPtr& scan) 
    {
        int en_yakin_duvar_indeksi = En_Yakin_Duvarin_Acisini_Bul(scan);
        
        ROS_INFO("En Yakin Duvar Indeksi: %d",en_yakin_duvar_indeksi);
        // en yakin duvar indisi onundeyse saga dogru 90 dondurelimn paralel olsun
        if((0 < en_yakin_duvar_indeksi && en_yakin_duvar_indeksi < 15) || (en_yakin_duvar_indeksi > 345 && en_yakin_duvar_indeksi < 359))
        {
            ROS_INFO("Saga 90 derece donuluyor!!!");
            Istenilen_Derece_Miktarinca_Don(true, 90);
        }
        else if( 255 < en_yakin_duvar_indeksi && en_yakin_duvar_indeksi < 285)
        {
            ROS_INFO("Saga 180 derece donuluyor!!!");
            Istenilen_Derece_Miktarinca_Don(true, 180);
        }
        else if( 165 < en_yakin_duvar_indeksi && en_yakin_duvar_indeksi < 195)
        {
            ROS_INFO("Sola 90 derece donuluyor!!!");
            Istenilen_Derece_Miktarinca_Don(false, 90);
        }
        else
        {
            ROS_INFO("Duvar zaten solumuzda dondurulmeye GEREK YOK!!!");
        }
        ROS_INFO("Baslangic dondurulmesi TAMAMLANDI!!!");
        
        init_ = true;
    }


    bool Kose_Var_Mi(TurtleBot robot) 
    {
        double kose_uzaklik_esigi = duvara_olan_uzaklik_ + parallel_band_ + 0.2;

        // 90 derece kose tespit edildi
        if (robot.on_uzaklik  < kose_uzaklik_esigi &&
            robot.sag_uzaklik > kose_uzaklik_esigi &&
            robot.sol_uzaklik < kose_uzaklik_esigi &&
            robot.sol_on_kose_uzaklik < kose_uzaklik_esigi) 
        {
            ROS_INFO("90 Derece kose tespit edildi - Saga dogru 90 derece donulmesi gerekiyor");
            return true;
        }

        // 270-derece kose tespit edildi
        else if (robot.on_uzaklik   > kose_uzaklik_esigi &&
                 robot.sag_uzaklik  > kose_uzaklik_esigi &&
                 robot.sol_uzaklik  < kose_uzaklik_esigi &&
                 robot.arka_uzaklik > kose_uzaklik_esigi && 
                 robot.sag_on_kose_uzaklik > kose_uzaklik_esigi &&
                 robot.sol_arka_kose_uzaklik < kose_uzaklik_esigi) 
        {
            ROS_INFO("270 Derece kose tespit edildi - Sol acik olana kadar ileri gidilip Sol dogru 90 derece donulmesi gerekiyor");
            return true;
        }

        return false;
    }


    // Duvari takip ederek paralel ilerleme saglayalim
    void Paralel_Mesafede_Duz_Ilerle(TurtleBot robot) 
    {
        if (Sol_Cok_Uzak(robot)) 
        {
            Sola_Don();
            ROS_INFO("Sol mesafe cok GENIS, sola yoneliyor");
        }
        else if (Sol_Cok_Yakin(robot)) 
        {
            Saga_Don();
            ROS_INFO("Sol mesafe cok DAR, saga yoneliyor");
        }
        else
        {
            Duz_Git();
            ROS_INFO("Paralel mesafede, DUZ ilerliyor");
        }

        cmd_pub_.publish(cmd_vel_);
    }

    bool Sol_Cok_Uzak(TurtleBot robot) 
    {
        double sol_duvara_olan_maks_uzaklik = duvara_olan_uzaklik_ + parallel_band_; 
        double on_tarafa_olan_uzaklik = duvara_olan_uzaklik_ + 0.2;

        return (robot.sol_uzaklik > sol_duvara_olan_maks_uzaklik ) &&
               (robot.sol_on_kose_uzaklik > sol_duvara_olan_maks_uzaklik + 0.2); 
    }

    bool Sol_Cok_Yakin(TurtleBot robot) 
    {
        double sol_duvara_olan_maks_uzaklik = duvara_olan_uzaklik_ + parallel_band_; 
        double on_tarafa_olan_uzaklik = duvara_olan_uzaklik_ + 0.5;

        return (robot.sol_uzaklik < on_tarafa_olan_uzaklik) &&
               (robot.sol_on_kose_uzaklik < on_tarafa_olan_uzaklik) &&
               (robot.sag_uzaklik > sol_duvara_olan_maks_uzaklik);
               
    }

    void Sola_Don() 
    {
        cmd_vel_.angular.z = 0.3;
        cmd_vel_.linear.x = 0.1; 
    }

    void Saga_Don() 
    {
        cmd_vel_.angular.z = -0.3;
        cmd_vel_.linear.x = 0.1;
    }

    void Duz_Git() 
    {
        cmd_vel_.angular.z = 0.0;
        cmd_vel_.linear.x = 0.2;
    }

    // Odometri verisi ile hedef kontrolu yapalim
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom) 
    {
        double x = odom->pose.pose.position.x;
        double y = odom->pose.pose.position.y;
        if (std::abs(x) < 1.0 && std::abs(y) < 1.0) 
        {
            ROS_INFO("Hedef pozisyona ulasildi - X: %f, Y: %f", x, y);
            hedefe_ulasildi_mi_ = true;
            ros::shutdown();
            return;
        }
    }

    // Robotu durduralim
    void Robotu_Durdur() 
    {
        cmd_vel_.linear.x = 0.0;
        cmd_vel_.angular.z = 0.0;
        cmd_pub_.publish(cmd_vel_);
        ROS_INFO("Robot Durdu");
    }
};

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "my_solver");

    MazeSolver solver;
    solver.spin();

    return 0;
}
