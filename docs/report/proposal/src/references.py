"""The proposal's references, in IEEE style. Keys are used as {{cite:key}} in parts/*.html; the
build numbers them in order of first citation. Entries marked VERIFIED were checked against the
publisher's record while drafting; the others should be confirmed on Google Scholar before
submission, as the briefing requires."""

REFERENCES = {
    # --- Road safety and driver attention ---
    "who2023":  # VERIFIED
        "World Health Organization, <i>Global Status Report on Road Safety 2023</i>. Geneva, "
        "Switzerland: WHO, 2023.",
    "ntsa2023":  # figures as published by NTSA and reported in the national press
        "National Transport and Safety Authority (NTSA), &ldquo;Road traffic crash statistics, "
        "2022&ndash;2023,&rdquo; Nairobi, Kenya, 2023, as reported in <i>The Star</i> (Kenya), Nov. 2023.",
    "klauer2006":
        "S. G. Klauer, T. A. Dingus, V. L. Neale, J. D. Sudweeks, and D. J. Ramsey, &ldquo;The impact "
        "of driver inattention on near-crash/crash risk: An analysis using the 100-car naturalistic "
        "driving study data,&rdquo; Nat. Highway Traffic Safety Admin., Washington, DC, USA, Tech. "
        "Rep. DOT HS 810 594, 2006.",
    "dingus2016":
        "T. A. Dingus, F. Guo, S. Lee, J. F. Antin, M. Perez, M. Buchanan-King, and J. Hankey, "
        "&ldquo;Driver crash risk factors and prevalence evaluation using naturalistic driving "
        "data,&rdquo; <i>Proc. Nat. Acad. Sci. USA</i>, vol. 113, no. 10, pp. 2636&ndash;2641, 2016.",
    # --- Head-up and AR displays ---
    "gabbard2014":
        "J. L. Gabbard, G. M. Fitch, and H. Kim, &ldquo;Behind the glass: Driver challenges and "
        "opportunities for AR automotive applications,&rdquo; <i>Proc. IEEE</i>, vol. 102, no. 2, "
        "pp. 124&ndash;136, Feb. 2014.",
    "kim2022":  # VERIFIED
        "H. Kim and J. L. Gabbard, &ldquo;Assessing distraction potential of augmented reality "
        "head-up displays for vehicle drivers,&rdquo; <i>Human Factors</i>, vol. 64, no. 5, "
        "pp. 852&ndash;865, 2022, doi: 10.1177/0018720819844845.",
    "bark2014":  # VERIFIED
        "K. Bark, C. Tran, K. Fujimura, and V. Ng-Thow-Hing, &ldquo;Personal Navi: Benefits of an "
        "augmented reality navigational aid using a see-thru 3D volumetric HUD,&rdquo; in <i>Proc. "
        "6th Int. Conf. Automotive User Interfaces and Interactive Vehicular Applications "
        "(AutomotiveUI &rsquo;14)</i>, Seattle, WA, USA, 2014, pp. 1&ndash;8, "
        "doi: 10.1145/2667317.2667329.",
    # --- Perception ---
    "redmon2016":
        "J. Redmon, S. Divvala, R. Girshick, and A. Farhadi, &ldquo;You only look once: Unified, "
        "real-time object detection,&rdquo; in <i>Proc. IEEE Conf. Computer Vision and Pattern "
        "Recognition (CVPR)</i>, Las Vegas, NV, USA, 2016, pp. 779&ndash;788.",
    "bolya2019":
        "D. Bolya, C. Zhou, F. Xiao, and Y. J. Lee, &ldquo;YOLACT: Real-time instance "
        "segmentation,&rdquo; in <i>Proc. IEEE/CVF Int. Conf. Computer Vision (ICCV)</i>, Seoul, "
        "South Korea, 2019, pp. 9157&ndash;9166.",
    "terven2023":
        "J. Terven, D.-M. C&oacute;rdova-Esparza, and J.-A. Romero-Gonz&aacute;lez, &ldquo;A "
        "comprehensive review of YOLO architectures in computer vision: From YOLOv1 to YOLOv8 and "
        "YOLO-NAS,&rdquo; <i>Mach. Learn. Knowl. Extr.</i>, vol. 5, no. 4, pp. 1680&ndash;1716, 2023.",
    "qin2020":
        "Z. Qin, H. Wang, and X. Li, &ldquo;Ultra fast structure-aware deep lane detection,&rdquo; in "
        "<i>Proc. European Conf. Computer Vision (ECCV)</i>, Glasgow, U.K., 2020, pp. 276&ndash;291.",
    "qin2024":  # VERIFIED
        "Z. Qin, P. Zhang, and X. Li, &ldquo;Ultra fast deep lane detection with hybrid anchor driven "
        "ordinal classification,&rdquo; <i>IEEE Trans. Pattern Anal. Mach. Intell.</i>, vol. 46, "
        "no. 5, pp. 2555&ndash;2568, May 2024.",
    "ranftl2022":
        "R. Ranftl, K. Lasinger, D. Hafner, K. Schindler, and V. Koltun, &ldquo;Towards robust "
        "monocular depth estimation: Mixing datasets for zero-shot cross-dataset transfer,&rdquo; "
        "<i>IEEE Trans. Pattern Anal. Mach. Intell.</i>, vol. 44, no. 3, pp. 1623&ndash;1637, Mar. 2022.",
    "ertler2020":  # VERIFIED
        "C. Ertler, J. Mislej, T. Ollmann, L. Porzi, G. Neuhold, and Y. Kuang, &ldquo;The Mapillary "
        "traffic sign dataset for detection and classification on a global scale,&rdquo; in "
        "<i>Proc. European Conf. Computer Vision (ECCV)</i>, 2020, pp. 68&ndash;84.",
    # --- Geometry, fusion and tracking ---
    "zhang2000":
        "Z. Zhang, &ldquo;A flexible new technique for camera calibration,&rdquo; <i>IEEE Trans. "
        "Pattern Anal. Mach. Intell.</i>, vol. 22, no. 11, pp. 1330&ndash;1334, Nov. 2000.",
    "hartley2004":
        "R. Hartley and A. Zisserman, <i>Multiple View Geometry in Computer Vision</i>, 2nd ed. "
        "Cambridge, U.K.: Cambridge Univ. Press, 2004.",
    "qi2018":
        "C. R. Qi, W. Liu, C. Wu, H. Su, and L. J. Guibas, &ldquo;Frustum PointNets for 3D object "
        "detection from RGB-D data,&rdquo; in <i>Proc. IEEE/CVF Conf. Computer Vision and Pattern "
        "Recognition (CVPR)</i>, Salt Lake City, UT, USA, 2018, pp. 918&ndash;927.",
    "vora2020":
        "S. Vora, A. H. Lang, B. Helou, and O. Beijbom, &ldquo;PointPainting: Sequential fusion for "
        "3D object detection,&rdquo; in <i>Proc. IEEE/CVF Conf. Computer Vision and Pattern "
        "Recognition (CVPR)</i>, 2020, pp. 4604&ndash;4612.",
    "kalman1960":
        "R. E. Kalman, &ldquo;A new approach to linear filtering and prediction problems,&rdquo; "
        "<i>Trans. ASME, J. Basic Eng.</i>, vol. 82, no. 1, pp. 35&ndash;45, 1960.",
    "julier2004":
        "S. J. Julier and J. K. Uhlmann, &ldquo;Unscented filtering and nonlinear estimation,&rdquo; "
        "<i>Proc. IEEE</i>, vol. 92, no. 3, pp. 401&ndash;422, Mar. 2004.",
    "blom1988":
        "H. A. P. Blom and Y. Bar-Shalom, &ldquo;The interacting multiple model algorithm for systems "
        "with Markovian switching coefficients,&rdquo; <i>IEEE Trans. Autom. Control</i>, vol. 33, "
        "no. 8, pp. 780&ndash;783, Aug. 1988.",
    "barshalom2001":
        "Y. Bar-Shalom, X. R. Li, and T. Kirubarajan, <i>Estimation with Applications to Tracking "
        "and Navigation</i>. New York, NY, USA: Wiley, 2001.",
    "thrun2005":
        "S. Thrun, W. Burgard, and D. Fox, <i>Probabilistic Robotics</i>. Cambridge, MA, USA: MIT "
        "Press, 2005.",
    "kuhn1955":
        "H. W. Kuhn, &ldquo;The Hungarian method for the assignment problem,&rdquo; <i>Naval Res. "
        "Logist. Quart.</i>, vol. 2, no. 1&ndash;2, pp. 83&ndash;97, 1955.",
    # --- Navigation ---
    "newson2009":
        "P. Newson and J. Krumm, &ldquo;Hidden Markov map matching through noise and "
        "sparseness,&rdquo; in <i>Proc. 17th ACM SIGSPATIAL Int. Conf. Advances in Geographic "
        "Information Systems</i>, Seattle, WA, USA, 2009, pp. 336&ndash;343.",
    "luxen2011":
        "D. Luxen and C. Vetter, &ldquo;Real-time routing with OpenStreetMap data,&rdquo; in "
        "<i>Proc. 19th ACM SIGSPATIAL Int. Conf. Advances in Geographic Information Systems</i>, "
        "Chicago, IL, USA, 2011, pp. 513&ndash;516.",
    "delling2011":  # VERIFIED
        "D. Delling, A. V. Goldberg, T. Pajor, and R. F. Werneck, &ldquo;Customizable route "
        "planning,&rdquo; in <i>Experimental Algorithms (SEA 2011)</i>, Lecture Notes in Computer "
        "Science, vol. 6630. Berlin, Germany: Springer, 2011, pp. 376&ndash;387.",
    # --- Collision avoidance and functional safety ---
    "lee1976":
        "D. N. Lee, &ldquo;A theory of visual control of braking based on information about "
        "time-to-collision,&rdquo; <i>Perception</i>, vol. 5, no. 4, pp. 437&ndash;459, 1976.",
    "cicchino2017":
        "J. B. Cicchino, &ldquo;Effectiveness of forward collision warning and autonomous emergency "
        "braking systems in reducing front-to-rear crash rates,&rdquo; <i>Accid. Anal. Prev.</i>, "
        "vol. 99, pp. 142&ndash;152, Feb. 2017.",
    "livox2024":
        "Livox Technology, <i>Livox Mid-360 User Manual</i>, v1.2. Shenzhen, China: Livox, Apr. "
        "2024.",
    "iso15765":
        "<i>Road Vehicles &mdash; Diagnostic Communication over Controller Area Network (DoCAN) &mdash; "
        "Part 4: Requirements for Emissions-Related Systems</i>, ISO 15765-4:2021, International "
        "Organization for Standardization, Geneva, Switzerland, 2021.",
    "iso26262":
        "<i>Road Vehicles &mdash; Functional Safety</i>, ISO 26262:2018, International Organization "
        "for Standardization, Geneva, Switzerland, 2018.",
}
