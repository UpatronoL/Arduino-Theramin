const PROGMEM uint16_t sines[4096] = {
2048 ,  2051 ,  2054 ,  2057 ,  2061 ,  2064 ,  2067 ,  2070 ,  2073 ,  2076 ,  2079 ,  2083 ,  2086 ,  2089 ,  2092 ,  2095 ,  2098 ,  2101 ,  2105 ,  2108 ,  2111 ,  2114 ,  2117 ,  2120 ,  2123 ,  2126 ,  2130 ,  2133 ,  2136 ,  2139 ,  2142 ,  2145 ,  2148 ,  2152 ,  2155 ,  2158 ,  2161 ,  2164 ,  2167 ,  2170 ,  2174 ,  2177 ,  2180 ,  2183 ,  2186 ,  2189 ,  2192 ,  2195 ,  2199 ,  2202 ,  2205 ,  2208 ,  2211 ,  2214 ,  2217 ,  2220 ,  2224 ,  2227 ,  2230 ,  2233 ,  2236 ,  2239 ,  2242 ,  2246 ,  2249 ,  2252 ,  2255 ,  2258 ,  2261 ,  2264 ,  2267 ,  2271 ,  2274 ,  2277 ,  2280 ,  2283 ,  2286 ,  2289 ,  2292 ,  2295 ,  2299 ,  2302 ,  2305 ,  2308 ,  2311 ,  2314 ,  2317 ,  2320 ,  2323 ,  2327 ,  2330 ,  2333 ,  2336 ,  2339 ,  2342 ,  2345 ,  2348 ,  2351 ,  2355 ,  2358 ,  2361 ,  2364 ,  2367 ,  2370 ,  2373 ,  2376 ,  2379 ,  2382 ,  2386 ,  2389 ,  2392 ,  2395 ,  2398 ,  2401 ,  2404 ,  2407 ,  2410 ,  2413 ,  2417 ,  2420 ,  2423 ,  2426 ,  2429 ,  2432 ,  2435 ,  2438 ,  2441 ,  2444 ,  2447 ,  2450 ,  2454 ,  2457 ,  2460 ,  2463 ,  2466 ,  2469 ,  2472 ,  2475 ,  2478 ,  2481 ,  2484 ,  2487 ,  2490 ,  2493 ,  2497 ,  2500 ,  2503 ,  2506 ,  2509 ,  2512 ,  2515 ,  2518 ,  2521 ,  2524 ,  2527 ,  2530 ,  2533 ,  2536 ,  2539 ,  2542 ,  2545 ,  2548 ,  2551 ,  2555 ,  2558 ,  2561 ,  2564 ,  2567 ,  2570 ,  2573 ,  2576 ,  2579 ,  2582 ,  2585 ,  2588 ,  2591 ,  2594 ,  2597 ,  2600 ,  2603 ,  2606 ,  2609 ,  2612 ,  2615 ,  2618 ,  2621 ,  2624 ,  2627 ,  2630 ,  2633 ,  2636 ,  2639 ,  2642 ,  2645 ,  2648 ,  2651 ,  2654 ,  2657 ,  2660 ,  2663 ,  2666 ,  2669 ,  2672 ,  2675 ,  2678 ,  2681 ,  2684 ,  2687 ,  2690 ,  2693 ,  2696 ,  2699 ,  2702 ,  2705 ,  2708 ,  2711 ,  2714 ,  2717 ,  2720 ,  2723 ,  2726 ,  2729 ,  2732 ,  2735 ,  2738 ,  2741 ,  2744 ,  2746 ,  2749 ,  2752 ,  2755 ,  2758 ,  2761 ,  2764 ,  2767 ,  2770 ,  2773 ,  2776 ,  2779 ,  2782 ,  2785 ,  2788 ,  2791 ,  2793 ,  2796 ,  2799 ,  2802 ,  2805 ,  2808 ,  2811 ,  2814 ,  2817 ,  2820 ,  2823 ,  2826 ,  2828 ,  2831 ,  2834 ,  2837 ,  2840 ,  2843 ,  2846 ,  2849 ,  2852 ,  2855 ,  2857 ,  2860 ,  2863 ,  2866 ,  2869 ,  2872 ,  2875 ,  2878 ,  2880 ,  2883 ,  2886 ,  2889 ,  2892 ,  2895 ,  2898 ,  2900 ,  2903 ,  2906 ,  2909 ,  2912 ,  2915 ,  2918 ,  2920 ,  2923 ,  2926 ,  2929 ,  2932 ,  2935 ,  2937 ,  2940 ,  2943 ,  2946 ,  2949 ,  2951 ,  2954 ,  2957 ,  2960 ,  2963 ,  2966 ,  2968 ,  2971 ,  2974 ,  2977 ,  2980 ,  2982 ,  2985 ,  2988 ,  2991 ,  2994 ,  2996 ,  2999 ,  3002 ,  3005 ,  3007 ,  3010 ,  3013 ,  3016 ,  3018 ,  3021 ,  3024 ,  3027 ,  3030 ,  3032 ,  3035 ,  3038 ,  3041 ,  3043 ,  3046 ,  3049 ,  3051 ,  3054 ,  3057 ,  3060 ,  3062 ,  3065 ,  3068 ,  3071 ,  3073 ,  3076 ,  3079 ,  3081 ,  3084 ,  3087 ,  3090 ,  3092 ,  3095 ,  3098 ,  3100 ,  3103 ,  3106 ,  3108 ,  3111 ,  3114 ,  3116 ,  3119 ,  3122 ,  3125 ,  3127 ,  3130 ,  3133 ,  3135 ,  3138 ,  3140 ,  3143 ,  3146 ,  3148 ,  3151 ,  3154 ,  3156 ,  3159 ,  3162 ,  3164 ,  3167 ,  3170 ,  3172 ,  3175 ,  3177 ,  3180 ,  3183 ,  3185 ,  3188 ,  3190 ,  3193 ,  3196 ,  3198 ,  3201 ,  3203 ,  3206 ,  3209 ,  3211 ,  3214 ,  3216 ,  3219 ,  3222 ,  3224 ,  3227 ,  3229 ,  3232 ,  3234 ,  3237 ,  3239 ,  3242 ,  3245 ,  3247 ,  3250 ,  3252 ,  3255 ,  3257 ,  3260 ,  3262 ,  3265 ,  3267 ,  3270 ,  3272 ,  3275 ,  3277 ,  3280 ,  3282 ,  3285 ,  3287 ,  3290 ,  3292 ,  3295 ,  3297 ,  3300 ,  3302 ,  3305 ,  3307 ,  3310 ,  3312 ,  3315 ,  3317 ,  3320 ,  3322 ,  3325 ,  3327 ,  3330 ,  3332 ,  3334 ,  3337 ,  3339 ,  3342 ,  3344 ,  3347 ,  3349 ,  3351 ,  3354 ,  3356 ,  3359 ,  3361 ,  3364 ,  3366 ,  3368 ,  3371 ,  3373 ,  3376 ,  3378 ,  3380 ,  3383 ,  3385 ,  3387 ,  3390 ,  3392 ,  3395 ,  3397 ,  3399 ,  3402 ,  3404 ,  3406 ,  3409 ,  3411 ,  3413 ,  3416 ,  3418 ,  3420 ,  3423 ,  3425 ,  3427 ,  3430 ,  3432 ,  3434 ,  3437 ,  3439 ,  3441 ,  3443 ,  3446 ,  3448 ,  3450 ,  3453 ,  3455 ,  3457 ,  3459 ,  3462 ,  3464 ,  3466 ,  3469 ,  3471 ,  3473 ,  3475 ,  3478 ,  3480 ,  3482 ,  3484 ,  3487 ,  3489 ,  3491 ,  3493 ,  3495 ,  3498 ,  3500 ,  3502 ,  3504 ,  3507 ,  3509 ,  3511 ,  3513 ,  3515 ,  3517 ,  3520 ,  3522 ,  3524 ,  3526 ,  3528 ,  3531 ,  3533 ,  3535 ,  3537 ,  3539 ,  3541 ,  3543 ,  3546 ,  3548 ,  3550 ,  3552 ,  3554 ,  3556 ,  3558 ,  3561 ,  3563 ,  3565 ,  3567 ,  3569 ,  3571 ,  3573 ,  3575 ,  3577 ,  3579 ,  3581 ,  3584 ,  3586 ,  3588 ,  3590 ,  3592 ,  3594 ,  3596 ,  3598 ,  3600 ,  3602 ,  3604 ,  3606 ,  3608 ,  3610 ,  3612 ,  3614 ,  3616 ,  3618 ,  3620 ,  3622 ,  3624 ,  3626 ,  3628 ,  3630 ,  3632 ,  3634 ,  3636 ,  3638 ,  3640 ,  3642 ,  3644 ,  3646 ,  3648 ,  3650 ,  3652 ,  3654 ,  3656 ,  3658 ,  3660 ,  3662 ,  3664 ,  3666 ,  3668 ,  3669 ,  3671 ,  3673 ,  3675 ,  3677 ,  3679 ,  3681 ,  3683 ,  3685 ,  3687 ,  3688 ,  3690 ,  3692 ,  3694 ,  3696 ,  3698 ,  3700 ,  3701 ,  3703 ,  3705 ,  3707 ,  3709 ,  3711 ,  3713 ,  3714 ,  3716 ,  3718 ,  3720 ,  3722 ,  3723 ,  3725 ,  3727 ,  3729 ,  3731 ,  3732 ,  3734 ,  3736 ,  3738 ,  3739 ,  3741 ,  3743 ,  3745 ,  3747 ,  3748 ,  3750 ,  3752 ,  3753 ,  3755 ,  3757 ,  3759 ,  3760 ,  3762 ,  3764 ,  3766 ,  3767 ,  3769 ,  3771 ,  3772 ,  3774 ,  3776 ,  3777 ,  3779 ,  3781 ,  3782 ,  3784 ,  3786 ,  3787 ,  3789 ,  3791 ,  3792 ,  3794 ,  3796 ,  3797 ,  3799 ,  3801 ,  3802 ,  3804 ,  3805 ,  3807 ,  3809 ,  3810 ,  3812 ,  3813 ,  3815 ,  3817 ,  3818 ,  3820 ,  3821 ,  3823 ,  3824 ,  3826 ,  3828 ,  3829 ,  3831 ,  3832 ,  3834 ,  3835 ,  3837 ,  3838 ,  3840 ,  3841 ,  3843 ,  3844 ,  3846 ,  3847 ,  3849 ,  3850 ,  3852 ,  3853 ,  3855 ,  3856 ,  3858 ,  3859 ,  3861 ,  3862 ,  3864 ,  3865 ,  3866 ,  3868 ,  3869 ,  3871 ,  3872 ,  3874 ,  3875 ,  3876 ,  3878 ,  3879 ,  3881 ,  3882 ,  3883 ,  3885 ,  3886 ,  3888 ,  3889 ,  3890 ,  3892 ,  3893 ,  3894 ,  3896 ,  3897 ,  3898 ,  3900 ,  3901 ,  3902 ,  3904 ,  3905 ,  3906 ,  3908 ,  3909 ,  3910 ,  3912 ,  3913 ,  3914 ,  3916 ,  3917 ,  3918 ,  3919 ,  3921 ,  3922 ,  3923 ,  3924 ,  3926 ,  3927 ,  3928 ,  3929 ,  3931 ,  3932 ,  3933 ,  3934 ,  3936 ,  3937 ,  3938 ,  3939 ,  3940 ,  3942 ,  3943 ,  3944 ,  3945 ,  3946 ,  3947 ,  3949 ,  3950 ,  3951 ,  3952 ,  3953 ,  3954 ,  3956 ,  3957 ,  3958 ,  3959 ,  3960 ,  3961 ,  3962 ,  3963 ,  3965 ,  3966 ,  3967 ,  3968 ,  3969 ,  3970 ,  3971 ,  3972 ,  3973 ,  3974 ,  3975 ,  3976 ,  3977 ,  3978 ,  3980 ,  3981 ,  3982 ,  3983 ,  3984 ,  3985 ,  3986 ,  3987 ,  3988 ,  3989 ,  3990 ,  3991 ,  3992 ,  3993 ,  3994 ,  3995 ,  3996 ,  3997 ,  3998 ,  3998 ,  3999 ,  4000 ,  4001 ,  4002 ,  4003 ,  4004 ,  4005 ,  4006 ,  4007 ,  4008 ,  4009 ,  4010 ,  4010 ,  4011 ,  4012 ,  4013 ,  4014 ,  4015 ,  4016 ,  4017 ,  4017 ,  4018 ,  4019 ,  4020 ,  4021 ,  4022 ,  4023 ,  4023 ,  4024 ,  4025 ,  4026 ,  4027 ,  4027 ,  4028 ,  4029 ,  4030 ,  4031 ,  4031 ,  4032 ,  4033 ,  4034 ,  4034 ,  4035 ,  4036 ,  4037 ,  4037 ,  4038 ,  4039 ,  4040 ,  4040 ,  4041 ,  4042 ,  4042 ,  4043 ,  4044 ,  4045 ,  4045 ,  4046 ,  4047 ,  4047 ,  4048 ,  4049 ,  4049 ,  4050 ,  4051 ,  4051 ,  4052 ,  4053 ,  4053 ,  4054 ,  4054 ,  4055 ,  4056 ,  4056 ,  4057 ,  4057 ,  4058 ,  4059 ,  4059 ,  4060 ,  4060 ,  4061 ,  4062 ,  4062 ,  4063 ,  4063 ,  4064 ,  4064 ,  4065 ,  4065 ,  4066 ,  4066 ,  4067 ,  4067 ,  4068 ,  4069 ,  4069 ,  4070 ,  4070 ,  4070 ,  4071 ,  4071 ,  4072 ,  4072 ,  4073 ,  4073 ,  4074 ,  4074 ,  4075 ,  4075 ,  4076 ,  4076 ,  4076 ,  4077 ,  4077 ,  4078 ,  4078 ,  4078 ,  4079 ,  4079 ,  4080 ,  4080 ,  4080 ,  4081 ,  4081 ,  4081 ,  4082 ,  4082 ,  4083 ,  4083 ,  4083 ,  4084 ,  4084 ,  4084 ,  4085 ,  4085 ,  4085 ,  4085 ,  4086 ,  4086 ,  4086 ,  4087 ,  4087 ,  4087 ,  4087 ,  4088 ,  4088 ,  4088 ,  4088 ,  4089 ,  4089 ,  4089 ,  4089 ,  4090 ,  4090 ,  4090 ,  4090 ,  4091 ,  4091 ,  4091 ,  4091 ,  4091 ,  4092 ,  4092 ,  4092 ,  4092 ,  4092 ,  4092 ,  4093 ,  4093 ,  4093 ,  4093 ,  4093 ,  4093 ,  4093 ,  4093 ,  4094 ,  4094 ,  4094 ,  4094 ,  4094 ,  4094 ,  4094 ,  4094 ,  4094 ,  4094 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4095 ,  4094 ,  4094 ,  4094 ,  4094 ,  4094 ,  4094 ,  4094 ,  4094 ,  4094 ,  4094 ,  4093 ,  4093 ,  4093 ,  4093 ,  4093 ,  4093 ,  4093 ,  4093 ,  4092 ,  4092 ,  4092 ,  4092 ,  4092 ,  4092 ,  4091 ,  4091 ,  4091 ,  4091 ,  4091 ,  4090 ,  4090 ,  4090 ,  4090 ,  4089 ,  4089 ,  4089 ,  4089 ,  4088 ,  4088 ,  4088 ,  4088 ,  4087 ,  4087 ,  4087 ,  4087 ,  4086 ,  4086 ,  4086 ,  4085 ,  4085 ,  4085 ,  4085 ,  4084 ,  4084 ,  4084 ,  4083 ,  4083 ,  4083 ,  4082 ,  4082 ,  4081 ,  4081 ,  4081 ,  4080 ,  4080 ,  4080 ,  4079 ,  4079 ,  4078 ,  4078 ,  4078 ,  4077 ,  4077 ,  4076 ,  4076 ,  4076 ,  4075 ,  4075 ,  4074 ,  4074 ,  4073 ,  4073 ,  4072 ,  4072 ,  4071 ,  4071 ,  4070 ,  4070 ,  4070 ,  4069 ,  4069 ,  4068 ,  4067 ,  4067 ,  4066 ,  4066 ,  4065 ,  4065 ,  4064 ,  4064 ,  4063 ,  4063 ,  4062 ,  4062 ,  4061 ,  4060 ,  4060 ,  4059 ,  4059 ,  4058 ,  4057 ,  4057 ,  4056 ,  4056 ,  4055 ,  4054 ,  4054 ,  4053 ,  4053 ,  4052 ,  4051 ,  4051 ,  4050 ,  4049 ,  4049 ,  4048 ,  4047 ,  4047 ,  4046 ,  4045 ,  4045 ,  4044 ,  4043 ,  4042 ,  4042 ,  4041 ,  4040 ,  4040 ,  4039 ,  4038 ,  4037 ,  4037 ,  4036 ,  4035 ,  4034 ,  4034 ,  4033 ,  4032 ,  4031 ,  4031 ,  4030 ,  4029 ,  4028 ,  4027 ,  4027 ,  4026 ,  4025 ,  4024 ,  4023 ,  4023 ,  4022 ,  4021 ,  4020 ,  4019 ,  4018 ,  4017 ,  4017 ,  4016 ,  4015 ,  4014 ,  4013 ,  4012 ,  4011 ,  4010 ,  4010 ,  4009 ,  4008 ,  4007 ,  4006 ,  4005 ,  4004 ,  4003 ,  4002 ,  4001 ,  4000 ,  3999 ,  3998 ,  3998 ,  3997 ,  3996 ,  3995 ,  3994 ,  3993 ,  3992 ,  3991 ,  3990 ,  3989 ,  3988 ,  3987 ,  3986 ,  3985 ,  3984 ,  3983 ,  3982 ,  3981 ,  3980 ,  3978 ,  3977 ,  3976 ,  3975 ,  3974 ,  3973 ,  3972 ,  3971 ,  3970 ,  3969 ,  3968 ,  3967 ,  3966 ,  3965 ,  3963 ,  3962 ,  3961 ,  3960 ,  3959 ,  3958 ,  3957 ,  3956 ,  3954 ,  3953 ,  3952 ,  3951 ,  3950 ,  3949 ,  3947 ,  3946 ,  3945 ,  3944 ,  3943 ,  3942 ,  3940 ,  3939 ,  3938 ,  3937 ,  3936 ,  3934 ,  3933 ,  3932 ,  3931 ,  3929 ,  3928 ,  3927 ,  3926 ,  3924 ,  3923 ,  3922 ,  3921 ,  3919 ,  3918 ,  3917 ,  3916 ,  3914 ,  3913 ,  3912 ,  3910 ,  3909 ,  3908 ,  3906 ,  3905 ,  3904 ,  3902 ,  3901 ,  3900 ,  3898 ,  3897 ,  3896 ,  3894 ,  3893 ,  3892 ,  3890 ,  3889 ,  3888 ,  3886 ,  3885 ,  3883 ,  3882 ,  3881 ,  3879 ,  3878 ,  3876 ,  3875 ,  3874 ,  3872 ,  3871 ,  3869 ,  3868 ,  3866 ,  3865 ,  3864 ,  3862 ,  3861 ,  3859 ,  3858 ,  3856 ,  3855 ,  3853 ,  3852 ,  3850 ,  3849 ,  3847 ,  3846 ,  3844 ,  3843 ,  3841 ,  3840 ,  3838 ,  3837 ,  3835 ,  3834 ,  3832 ,  3831 ,  3829 ,  3828 ,  3826 ,  3824 ,  3823 ,  3821 ,  3820 ,  3818 ,  3817 ,  3815 ,  3813 ,  3812 ,  3810 ,  3809 ,  3807 ,  3805 ,  3804 ,  3802 ,  3801 ,  3799 ,  3797 ,  3796 ,  3794 ,  3792 ,  3791 ,  3789 ,  3787 ,  3786 ,  3784 ,  3782 ,  3781 ,  3779 ,  3777 ,  3776 ,  3774 ,  3772 ,  3771 ,  3769 ,  3767 ,  3766 ,  3764 ,  3762 ,  3760 ,  3759 ,  3757 ,  3755 ,  3753 ,  3752 ,  3750 ,  3748 ,  3747 ,  3745 ,  3743 ,  3741 ,  3739 ,  3738 ,  3736 ,  3734 ,  3732 ,  3731 ,  3729 ,  3727 ,  3725 ,  3723 ,  3722 ,  3720 ,  3718 ,  3716 ,  3714 ,  3713 ,  3711 ,  3709 ,  3707 ,  3705 ,  3703 ,  3701 ,  3700 ,  3698 ,  3696 ,  3694 ,  3692 ,  3690 ,  3688 ,  3687 ,  3685 ,  3683 ,  3681 ,  3679 ,  3677 ,  3675 ,  3673 ,  3671 ,  3669 ,  3668 ,  3666 ,  3664 ,  3662 ,  3660 ,  3658 ,  3656 ,  3654 ,  3652 ,  3650 ,  3648 ,  3646 ,  3644 ,  3642 ,  3640 ,  3638 ,  3636 ,  3634 ,  3632 ,  3630 ,  3628 ,  3626 ,  3624 ,  3622 ,  3620 ,  3618 ,  3616 ,  3614 ,  3612 ,  3610 ,  3608 ,  3606 ,  3604 ,  3602 ,  3600 ,  3598 ,  3596 ,  3594 ,  3592 ,  3590 ,  3588 ,  3586 ,  3584 ,  3581 ,  3579 ,  3577 ,  3575 ,  3573 ,  3571 ,  3569 ,  3567 ,  3565 ,  3563 ,  3561 ,  3558 ,  3556 ,  3554 ,  3552 ,  3550 ,  3548 ,  3546 ,  3543 ,  3541 ,  3539 ,  3537 ,  3535 ,  3533 ,  3531 ,  3528 ,  3526 ,  3524 ,  3522 ,  3520 ,  3517 ,  3515 ,  3513 ,  3511 ,  3509 ,  3507 ,  3504 ,  3502 ,  3500 ,  3498 ,  3495 ,  3493 ,  3491 ,  3489 ,  3487 ,  3484 ,  3482 ,  3480 ,  3478 ,  3475 ,  3473 ,  3471 ,  3469 ,  3466 ,  3464 ,  3462 ,  3459 ,  3457 ,  3455 ,  3453 ,  3450 ,  3448 ,  3446 ,  3443 ,  3441 ,  3439 ,  3437 ,  3434 ,  3432 ,  3430 ,  3427 ,  3425 ,  3423 ,  3420 ,  3418 ,  3416 ,  3413 ,  3411 ,  3409 ,  3406 ,  3404 ,  3402 ,  3399 ,  3397 ,  3395 ,  3392 ,  3390 ,  3387 ,  3385 ,  3383 ,  3380 ,  3378 ,  3376 ,  3373 ,  3371 ,  3368 ,  3366 ,  3364 ,  3361 ,  3359 ,  3356 ,  3354 ,  3351 ,  3349 ,  3347 ,  3344 ,  3342 ,  3339 ,  3337 ,  3334 ,  3332 ,  3330 ,  3327 ,  3325 ,  3322 ,  3320 ,  3317 ,  3315 ,  3312 ,  3310 ,  3307 ,  3305 ,  3302 ,  3300 ,  3297 ,  3295 ,  3292 ,  3290 ,  3287 ,  3285 ,  3282 ,  3280 ,  3277 ,  3275 ,  3272 ,  3270 ,  3267 ,  3265 ,  3262 ,  3260 ,  3257 ,  3255 ,  3252 ,  3250 ,  3247 ,  3245 ,  3242 ,  3239 ,  3237 ,  3234 ,  3232 ,  3229 ,  3227 ,  3224 ,  3222 ,  3219 ,  3216 ,  3214 ,  3211 ,  3209 ,  3206 ,  3203 ,  3201 ,  3198 ,  3196 ,  3193 ,  3190 ,  3188 ,  3185 ,  3183 ,  3180 ,  3177 ,  3175 ,  3172 ,  3170 ,  3167 ,  3164 ,  3162 ,  3159 ,  3156 ,  3154 ,  3151 ,  3148 ,  3146 ,  3143 ,  3140 ,  3138 ,  3135 ,  3133 ,  3130 ,  3127 ,  3125 ,  3122 ,  3119 ,  3116 ,  3114 ,  3111 ,  3108 ,  3106 ,  3103 ,  3100 ,  3098 ,  3095 ,  3092 ,  3090 ,  3087 ,  3084 ,  3081 ,  3079 ,  3076 ,  3073 ,  3071 ,  3068 ,  3065 ,  3062 ,  3060 ,  3057 ,  3054 ,  3051 ,  3049 ,  3046 ,  3043 ,  3041 ,  3038 ,  3035 ,  3032 ,  3030 ,  3027 ,  3024 ,  3021 ,  3018 ,  3016 ,  3013 ,  3010 ,  3007 ,  3005 ,  3002 ,  2999 ,  2996 ,  2994 ,  2991 ,  2988 ,  2985 ,  2982 ,  2980 ,  2977 ,  2974 ,  2971 ,  2968 ,  2966 ,  2963 ,  2960 ,  2957 ,  2954 ,  2951 ,  2949 ,  2946 ,  2943 ,  2940 ,  2937 ,  2935 ,  2932 ,  2929 ,  2926 ,  2923 ,  2920 ,  2918 ,  2915 ,  2912 ,  2909 ,  2906 ,  2903 ,  2900 ,  2898 ,  2895 ,  2892 ,  2889 ,  2886 ,  2883 ,  2880 ,  2878 ,  2875 ,  2872 ,  2869 ,  2866 ,  2863 ,  2860 ,  2857 ,  2855 ,  2852 ,  2849 ,  2846 ,  2843 ,  2840 ,  2837 ,  2834 ,  2831 ,  2828 ,  2826 ,  2823 ,  2820 ,  2817 ,  2814 ,  2811 ,  2808 ,  2805 ,  2802 ,  2799 ,  2796 ,  2793 ,  2791 ,  2788 ,  2785 ,  2782 ,  2779 ,  2776 ,  2773 ,  2770 ,  2767 ,  2764 ,  2761 ,  2758 ,  2755 ,  2752 ,  2749 ,  2746 ,  2744 ,  2741 ,  2738 ,  2735 ,  2732 ,  2729 ,  2726 ,  2723 ,  2720 ,  2717 ,  2714 ,  2711 ,  2708 ,  2705 ,  2702 ,  2699 ,  2696 ,  2693 ,  2690 ,  2687 ,  2684 ,  2681 ,  2678 ,  2675 ,  2672 ,  2669 ,  2666 ,  2663 ,  2660 ,  2657 ,  2654 ,  2651 ,  2648 ,  2645 ,  2642 ,  2639 ,  2636 ,  2633 ,  2630 ,  2627 ,  2624 ,  2621 ,  2618 ,  2615 ,  2612 ,  2609 ,  2606 ,  2603 ,  2600 ,  2597 ,  2594 ,  2591 ,  2588 ,  2585 ,  2582 ,  2579 ,  2576 ,  2573 ,  2570 ,  2567 ,  2564 ,  2561 ,  2558 ,  2555 ,  2551 ,  2548 ,  2545 ,  2542 ,  2539 ,  2536 ,  2533 ,  2530 ,  2527 ,  2524 ,  2521 ,  2518 ,  2515 ,  2512 ,  2509 ,  2506 ,  2503 ,  2500 ,  2497 ,  2493 ,  2490 ,  2487 ,  2484 ,  2481 ,  2478 ,  2475 ,  2472 ,  2469 ,  2466 ,  2463 ,  2460 ,  2457 ,  2454 ,  2450 ,  2447 ,  2444 ,  2441 ,  2438 ,  2435 ,  2432 ,  2429 ,  2426 ,  2423 ,  2420 ,  2417 ,  2413 ,  2410 ,  2407 ,  2404 ,  2401 ,  2398 ,  2395 ,  2392 ,  2389 ,  2386 ,  2382 ,  2379 ,  2376 ,  2373 ,  2370 ,  2367 ,  2364 ,  2361 ,  2358 ,  2355 ,  2351 ,  2348 ,  2345 ,  2342 ,  2339 ,  2336 ,  2333 ,  2330 ,  2327 ,  2323 ,  2320 ,  2317 ,  2314 ,  2311 ,  2308 ,  2305 ,  2302 ,  2299 ,  2295 ,  2292 ,  2289 ,  2286 ,  2283 ,  2280 ,  2277 ,  2274 ,  2271 ,  2267 ,  2264 ,  2261 ,  2258 ,  2255 ,  2252 ,  2249 ,  2246 ,  2242 ,  2239 ,  2236 ,  2233 ,  2230 ,  2227 ,  2224 ,  2220 ,  2217 ,  2214 ,  2211 ,  2208 ,  2205 ,  2202 ,  2199 ,  2195 ,  2192 ,  2189 ,  2186 ,  2183 ,  2180 ,  2177 ,  2174 ,  2170 ,  2167 ,  2164 ,  2161 ,  2158 ,  2155 ,  2152 ,  2148 ,  2145 ,  2142 ,  2139 ,  2136 ,  2133 ,  2130 ,  2126 ,  2123 ,  2120 ,  2117 ,  2114 ,  2111 ,  2108 ,  2105 ,  2101 ,  2098 ,  2095 ,  2092 ,  2089 ,  2086 ,  2083 ,  2079 ,  2076 ,  2073 ,  2070 ,  2067 ,  2064 ,  2061 ,  2057 ,  2054 ,  2051 ,  2048 ,  2045 ,  2042 ,  2039 ,  2035 ,  2032 ,  2029 ,  2026 ,  2023 ,  2020 ,  2017 ,  2013 ,  2010 ,  2007 ,  2004 ,  2001 ,  1998 ,  1995 ,  1991 ,  1988 ,  1985 ,  1982 ,  1979 ,  1976 ,  1973 ,  1970 ,  1966 ,  1963 ,  1960 ,  1957 ,  1954 ,  1951 ,  1948 ,  1944 ,  1941 ,  1938 ,  1935 ,  1932 ,  1929 ,  1926 ,  1922 ,  1919 ,  1916 ,  1913 ,  1910 ,  1907 ,  1904 ,  1901 ,  1897 ,  1894 ,  1891 ,  1888 ,  1885 ,  1882 ,  1879 ,  1876 ,  1872 ,  1869 ,  1866 ,  1863 ,  1860 ,  1857 ,  1854 ,  1850 ,  1847 ,  1844 ,  1841 ,  1838 ,  1835 ,  1832 ,  1829 ,  1825 ,  1822 ,  1819 ,  1816 ,  1813 ,  1810 ,  1807 ,  1804 ,  1801 ,  1797 ,  1794 ,  1791 ,  1788 ,  1785 ,  1782 ,  1779 ,  1776 ,  1773 ,  1769 ,  1766 ,  1763 ,  1760 ,  1757 ,  1754 ,  1751 ,  1748 ,  1745 ,  1741 ,  1738 ,  1735 ,  1732 ,  1729 ,  1726 ,  1723 ,  1720 ,  1717 ,  1714 ,  1710 ,  1707 ,  1704 ,  1701 ,  1698 ,  1695 ,  1692 ,  1689 ,  1686 ,  1683 ,  1679 ,  1676 ,  1673 ,  1670 ,  1667 ,  1664 ,  1661 ,  1658 ,  1655 ,  1652 ,  1649 ,  1646 ,  1642 ,  1639 ,  1636 ,  1633 ,  1630 ,  1627 ,  1624 ,  1621 ,  1618 ,  1615 ,  1612 ,  1609 ,  1606 ,  1603 ,  1599 ,  1596 ,  1593 ,  1590 ,  1587 ,  1584 ,  1581 ,  1578 ,  1575 ,  1572 ,  1569 ,  1566 ,  1563 ,  1560 ,  1557 ,  1554 ,  1551 ,  1548 ,  1545 ,  1541 ,  1538 ,  1535 ,  1532 ,  1529 ,  1526 ,  1523 ,  1520 ,  1517 ,  1514 ,  1511 ,  1508 ,  1505 ,  1502 ,  1499 ,  1496 ,  1493 ,  1490 ,  1487 ,  1484 ,  1481 ,  1478 ,  1475 ,  1472 ,  1469 ,  1466 ,  1463 ,  1460 ,  1457 ,  1454 ,  1451 ,  1448 ,  1445 ,  1442 ,  1439 ,  1436 ,  1433 ,  1430 ,  1427 ,  1424 ,  1421 ,  1418 ,  1415 ,  1412 ,  1409 ,  1406 ,  1403 ,  1400 ,  1397 ,  1394 ,  1391 ,  1388 ,  1385 ,  1382 ,  1379 ,  1376 ,  1373 ,  1370 ,  1367 ,  1364 ,  1361 ,  1358 ,  1355 ,  1352 ,  1350 ,  1347 ,  1344 ,  1341 ,  1338 ,  1335 ,  1332 ,  1329 ,  1326 ,  1323 ,  1320 ,  1317 ,  1314 ,  1311 ,  1308 ,  1305 ,  1303 ,  1300 ,  1297 ,  1294 ,  1291 ,  1288 ,  1285 ,  1282 ,  1279 ,  1276 ,  1273 ,  1270 ,  1268 ,  1265 ,  1262 ,  1259 ,  1256 ,  1253 ,  1250 ,  1247 ,  1244 ,  1241 ,  1239 ,  1236 ,  1233 ,  1230 ,  1227 ,  1224 ,  1221 ,  1218 ,  1216 ,  1213 ,  1210 ,  1207 ,  1204 ,  1201 ,  1198 ,  1196 ,  1193 ,  1190 ,  1187 ,  1184 ,  1181 ,  1178 ,  1176 ,  1173 ,  1170 ,  1167 ,  1164 ,  1161 ,  1159 ,  1156 ,  1153 ,  1150 ,  1147 ,  1145 ,  1142 ,  1139 ,  1136 ,  1133 ,  1130 ,  1128 ,  1125 ,  1122 ,  1119 ,  1116 ,  1114 ,  1111 ,  1108 ,  1105 ,  1102 ,  1100 ,  1097 ,  1094 ,  1091 ,  1089 ,  1086 ,  1083 ,  1080 ,  1078 ,  1075 ,  1072 ,  1069 ,  1066 ,  1064 ,  1061 ,  1058 ,  1055 ,  1053 ,  1050 ,  1047 ,  1045 ,  1042 ,  1039 ,  1036 ,  1034 ,  1031 ,  1028 ,  1025 ,  1023 ,  1020 ,  1017 ,  1015 ,  1012 ,  1009 ,  1006 ,  1004 ,  1001 ,  998 ,  996 ,  993 ,  990 ,  988 ,  985 ,  982 ,  980 ,  977 ,  974 ,  971 ,  969 ,  966 ,  963 ,  961 ,  958 ,  956 ,  953 ,  950 ,  948 ,  945 ,  942 ,  940 ,  937 ,  934 ,  932 ,  929 ,  926 ,  924 ,  921 ,  919 ,  916 ,  913 ,  911 ,  908 ,  906 ,  903 ,  900 ,  898 ,  895 ,  893 ,  890 ,  887 ,  885 ,  882 ,  880 ,  877 ,  874 ,  872 ,  869 ,  867 ,  864 ,  862 ,  859 ,  857 ,  854 ,  851 ,  849 ,  846 ,  844 ,  841 ,  839 ,  836 ,  834 ,  831 ,  829 ,  826 ,  824 ,  821 ,  819 ,  816 ,  814 ,  811 ,  809 ,  806 ,  804 ,  801 ,  799 ,  796 ,  794 ,  791 ,  789 ,  786 ,  784 ,  781 ,  779 ,  776 ,  774 ,  771 ,  769 ,  766 ,  764 ,  762 ,  759 ,  757 ,  754 ,  752 ,  749 ,  747 ,  745 ,  742 ,  740 ,  737 ,  735 ,  732 ,  730 ,  728 ,  725 ,  723 ,  720 ,  718 ,  716 ,  713 ,  711 ,  709 ,  706 ,  704 ,  701 ,  699 ,  697 ,  694 ,  692 ,  690 ,  687 ,  685 ,  683 ,  680 ,  678 ,  676 ,  673 ,  671 ,  669 ,  666 ,  664 ,  662 ,  659 ,  657 ,  655 ,  653 ,  650 ,  648 ,  646 ,  643 ,  641 ,  639 ,  637 ,  634 ,  632 ,  630 ,  627 ,  625 ,  623 ,  621 ,  618 ,  616 ,  614 ,  612 ,  609 ,  607 ,  605 ,  603 ,  601 ,  598 ,  596 ,  594 ,  592 ,  589 ,  587 ,  585 ,  583 ,  581 ,  579 ,  576 ,  574 ,  572 ,  570 ,  568 ,  565 ,  563 ,  561 ,  559 ,  557 ,  555 ,  553 ,  550 ,  548 ,  546 ,  544 ,  542 ,  540 ,  538 ,  535 ,  533 ,  531 ,  529 ,  527 ,  525 ,  523 ,  521 ,  519 ,  517 ,  515 ,  512 ,  510 ,  508 ,  506 ,  504 ,  502 ,  500 ,  498 ,  496 ,  494 ,  492 ,  490 ,  488 ,  486 ,  484 ,  482 ,  480 ,  478 ,  476 ,  474 ,  472 ,  470 ,  468 ,  466 ,  464 ,  462 ,  460 ,  458 ,  456 ,  454 ,  452 ,  450 ,  448 ,  446 ,  444 ,  442 ,  440 ,  438 ,  436 ,  434 ,  432 ,  430 ,  428 ,  427 ,  425 ,  423 ,  421 ,  419 ,  417 ,  415 ,  413 ,  411 ,  409 ,  408 ,  406 ,  404 ,  402 ,  400 ,  398 ,  396 ,  395 ,  393 ,  391 ,  389 ,  387 ,  385 ,  383 ,  382 ,  380 ,  378 ,  376 ,  374 ,  373 ,  371 ,  369 ,  367 ,  365 ,  364 ,  362 ,  360 ,  358 ,  357 ,  355 ,  353 ,  351 ,  349 ,  348 ,  346 ,  344 ,  343 ,  341 ,  339 ,  337 ,  336 ,  334 ,  332 ,  330 ,  329 ,  327 ,  325 ,  324 ,  322 ,  320 ,  319 ,  317 ,  315 ,  314 ,  312 ,  310 ,  309 ,  307 ,  305 ,  304 ,  302 ,  300 ,  299 ,  297 ,  295 ,  294 ,  292 ,  291 ,  289 ,  287 ,  286 ,  284 ,  283 ,  281 ,  279 ,  278 ,  276 ,  275 ,  273 ,  272 ,  270 ,  268 ,  267 ,  265 ,  264 ,  262 ,  261 ,  259 ,  258 ,  256 ,  255 ,  253 ,  252 ,  250 ,  249 ,  247 ,  246 ,  244 ,  243 ,  241 ,  240 ,  238 ,  237 ,  235 ,  234 ,  232 ,  231 ,  230 ,  228 ,  227 ,  225 ,  224 ,  222 ,  221 ,  220 ,  218 ,  217 ,  215 ,  214 ,  213 ,  211 ,  210 ,  208 ,  207 ,  206 ,  204 ,  203 ,  202 ,  200 ,  199 ,  198 ,  196 ,  195 ,  194 ,  192 ,  191 ,  190 ,  188 ,  187 ,  186 ,  184 ,  183 ,  182 ,  180 ,  179 ,  178 ,  177 ,  175 ,  174 ,  173 ,  172 ,  170 ,  169 ,  168 ,  167 ,  165 ,  164 ,  163 ,  162 ,  160 ,  159 ,  158 ,  157 ,  156 ,  154 ,  153 ,  152 ,  151 ,  150 ,  149 ,  147 ,  146 ,  145 ,  144 ,  143 ,  142 ,  140 ,  139 ,  138 ,  137 ,  136 ,  135 ,  134 ,  133 ,  131 ,  130 ,  129 ,  128 ,  127 ,  126 ,  125 ,  124 ,  123 ,  122 ,  121 ,  120 ,  119 ,  118 ,  116 ,  115 ,  114 ,  113 ,  112 ,  111 ,  110 ,  109 ,  108 ,  107 ,  106 ,  105 ,  104 ,  103 ,  102 ,  101 ,  100 ,  99 ,  98 ,  98 ,  97 ,  96 ,  95 ,  94 ,  93 ,  92 ,  91 ,  90 ,  89 ,  88 ,  87 ,  86 ,  86 ,  85 ,  84 ,  83 ,  82 ,  81 ,  80 ,  79 ,  79 ,  78 ,  77 ,  76 ,  75 ,  74 ,  73 ,  73 ,  72 ,  71 ,  70 ,  69 ,  69 ,  68 ,  67 ,  66 ,  65 ,  65 ,  64 ,  63 ,  62 ,  62 ,  61 ,  60 ,  59 ,  59 ,  58 ,  57 ,  56 ,  56 ,  55 ,  54 ,  54 ,  53 ,  52 ,  51 ,  51 ,  50 ,  49 ,  49 ,  48 ,  47 ,  47 ,  46 ,  45 ,  45 ,  44 ,  43 ,  43 ,  42 ,  42 ,  41 ,  40 ,  40 ,  39 ,  39 ,  38 ,  37 ,  37 ,  36 ,  36 ,  35 ,  34 ,  34 ,  33 ,  33 ,  32 ,  32 ,  31 ,  31 ,  30 ,  30 ,  29 ,  29 ,  28 ,  27 ,  27 ,  26 ,  26 ,  26 ,  25 ,  25 ,  24 ,  24 ,  23 ,  23 ,  22 ,  22 ,  21 ,  21 ,  20 ,  20 ,  20 ,  19 ,  19 ,  18 ,  18 ,  18 ,  17 ,  17 ,  16 ,  16 ,  16 ,  15 ,  15 ,  15 ,  14 ,  14 ,  13 ,  13 ,  13 ,  12 ,  12 ,  12 ,  11 ,  11 ,  11 ,  11 ,  10 ,  10 ,  10 ,  9 ,  9 ,  9 ,  9 ,  8 ,  8 ,  8 ,  8 ,  7 ,  7 ,  7 ,  7 ,  6 ,  6 ,  6 ,  6 ,  5 ,  5 ,  5 ,  5 ,  5 ,  4 ,  4 ,  4 ,  4 ,  4 ,  4 ,  3 ,  3 ,  3 ,  3 ,  3 ,  3 ,  3 ,  3 ,  2 ,  2 ,  2 ,  2 ,  2 ,  2 ,  2 ,  2 ,  2 ,  2 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  1 ,  2 ,  2 ,  2 ,  2 ,  2 ,  2 ,  2 ,  2 ,  2 ,  2 ,  3 ,  3 ,  3 ,  3 ,  3 ,  3 ,  3 ,  3 ,  4 ,  4 ,  4 ,  4 ,  4 ,  4 ,  5 ,  5 ,  5 ,  5 ,  5 ,  6 ,  6 ,  6 ,  6 ,  7 ,  7 ,  7 ,  7 ,  8 ,  8 ,  8 ,  8 ,  9 ,  9 ,  9 ,  9 ,  10 ,  10 ,  10 ,  11 ,  11 ,  11 ,  11 ,  12 ,  12 ,  12 ,  13 ,  13 ,  13 ,  14 ,  14 ,  15 ,  15 ,  15 ,  16 ,  16 ,  16 ,  17 ,  17 ,  18 ,  18 ,  18 ,  19 ,  19 ,  20 ,  20 ,  20 ,  21 ,  21 ,  22 ,  22 ,  23 ,  23 ,  24 ,  24 ,  25 ,  25 ,  26 ,  26 ,  26 ,  27 ,  27 ,  28 ,  29 ,  29 ,  30 ,  30 ,  31 ,  31 ,  32 ,  32 ,  33 ,  33 ,  34 ,  34 ,  35 ,  36 ,  36 ,  37 ,  37 ,  38 ,  39 ,  39 ,  40 ,  40 ,  41 ,  42 ,  42 ,  43 ,  43 ,  44 ,  45 ,  45 ,  46 ,  47 ,  47 ,  48 ,  49 ,  49 ,  50 ,  51 ,  51 ,  52 ,  53 ,  54 ,  54 ,  55 ,  56 ,  56 ,  57 ,  58 ,  59 ,  59 ,  60 ,  61 ,  62 ,  62 ,  63 ,  64 ,  65 ,  65 ,  66 ,  67 ,  68 ,  69 ,  69 ,  70 ,  71 ,  72 ,  73 ,  73 ,  74 ,  75 ,  76 ,  77 ,  78 ,  79 ,  79 ,  80 ,  81 ,  82 ,  83 ,  84 ,  85 ,  86 ,  86 ,  87 ,  88 ,  89 ,  90 ,  91 ,  92 ,  93 ,  94 ,  95 ,  96 ,  97 ,  98 ,  98 ,  99 ,  100 ,  101 ,  102 ,  103 ,  104 ,  105 ,  106 ,  107 ,  108 ,  109 ,  110 ,  111 ,  112 ,  113 ,  114 ,  115 ,  116 ,  118 ,  119 ,  120 ,  121 ,  122 ,  123 ,  124 ,  125 ,  126 ,  127 ,  128 ,  129 ,  130 ,  131 ,  133 ,  134 ,  135 ,  136 ,  137 ,  138 ,  139 ,  140 ,  142 ,  143 ,  144 ,  145 ,  146 ,  147 ,  149 ,  150 ,  151 ,  152 ,  153 ,  154 ,  156 ,  157 ,  158 ,  159 ,  160 ,  162 ,  163 ,  164 ,  165 ,  167 ,  168 ,  169 ,  170 ,  172 ,  173 ,  174 ,  175 ,  177 ,  178 ,  179 ,  180 ,  182 ,  183 ,  184 ,  186 ,  187 ,  188 ,  190 ,  191 ,  192 ,  194 ,  195 ,  196 ,  198 ,  199 ,  200 ,  202 ,  203 ,  204 ,  206 ,  207 ,  208 ,  210 ,  211 ,  213 ,  214 ,  215 ,  217 ,  218 ,  220 ,  221 ,  222 ,  224 ,  225 ,  227 ,  228 ,  230 ,  231 ,  232 ,  234 ,  235 ,  237 ,  238 ,  240 ,  241 ,  243 ,  244 ,  246 ,  247 ,  249 ,  250 ,  252 ,  253 ,  255 ,  256 ,  258 ,  259 ,  261 ,  262 ,  264 ,  265 ,  267 ,  268 ,  270 ,  272 ,  273 ,  275 ,  276 ,  278 ,  279 ,  281 ,  283 ,  284 ,  286 ,  287 ,  289 ,  291 ,  292 ,  294 ,  295 ,  297 ,  299 ,  300 ,  302 ,  304 ,  305 ,  307 ,  309 ,  310 ,  312 ,  314 ,  315 ,  317 ,  319 ,  320 ,  322 ,  324 ,  325 ,  327 ,  329 ,  330 ,  332 ,  334 ,  336 ,  337 ,  339 ,  341 ,  343 ,  344 ,  346 ,  348 ,  349 ,  351 ,  353 ,  355 ,  357 ,  358 ,  360 ,  362 ,  364 ,  365 ,  367 ,  369 ,  371 ,  373 ,  374 ,  376 ,  378 ,  380 ,  382 ,  383 ,  385 ,  387 ,  389 ,  391 ,  393 ,  395 ,  396 ,  398 ,  400 ,  402 ,  404 ,  406 ,  408 ,  409 ,  411 ,  413 ,  415 ,  417 ,  419 ,  421 ,  423 ,  425 ,  427 ,  428 ,  430 ,  432 ,  434 ,  436 ,  438 ,  440 ,  442 ,  444 ,  446 ,  448 ,  450 ,  452 ,  454 ,  456 ,  458 ,  460 ,  462 ,  464 ,  466 ,  468 ,  470 ,  472 ,  474 ,  476 ,  478 ,  480 ,  482 ,  484 ,  486 ,  488 ,  490 ,  492 ,  494 ,  496 ,  498 ,  500 ,  502 ,  504 ,  506 ,  508 ,  510 ,  512 ,  515 ,  517 ,  519 ,  521 ,  523 ,  525 ,  527 ,  529 ,  531 ,  533 ,  535 ,  538 ,  540 ,  542 ,  544 ,  546 ,  548 ,  550 ,  553 ,  555 ,  557 ,  559 ,  561 ,  563 ,  565 ,  568 ,  570 ,  572 ,  574 ,  576 ,  579 ,  581 ,  583 ,  585 ,  587 ,  589 ,  592 ,  594 ,  596 ,  598 ,  601 ,  603 ,  605 ,  607 ,  609 ,  612 ,  614 ,  616 ,  618 ,  621 ,  623 ,  625 ,  627 ,  630 ,  632 ,  634 ,  637 ,  639 ,  641 ,  643 ,  646 ,  648 ,  650 ,  653 ,  655 ,  657 ,  659 ,  662 ,  664 ,  666 ,  669 ,  671 ,  673 ,  676 ,  678 ,  680 ,  683 ,  685 ,  687 ,  690 ,  692 ,  694 ,  697 ,  699 ,  701 ,  704 ,  706 ,  709 ,  711 ,  713 ,  716 ,  718 ,  720 ,  723 ,  725 ,  728 ,  730 ,  732 ,  735 ,  737 ,  740 ,  742 ,  745 ,  747 ,  749 ,  752 ,  754 ,  757 ,  759 ,  762 ,  764 ,  766 ,  769 ,  771 ,  774 ,  776 ,  779 ,  781 ,  784 ,  786 ,  789 ,  791 ,  794 ,  796 ,  799 ,  801 ,  804 ,  806 ,  809 ,  811 ,  814 ,  816 ,  819 ,  821 ,  824 ,  826 ,  829 ,  831 ,  834 ,  836 ,  839 ,  841 ,  844 ,  846 ,  849 ,  851 ,  854 ,  857 ,  859 ,  862 ,  864 ,  867 ,  869 ,  872 ,  874 ,  877 ,  880 ,  882 ,  885 ,  887 ,  890 ,  893 ,  895 ,  898 ,  900 ,  903 ,  906 ,  908 ,  911 ,  913 ,  916 ,  919 ,  921 ,  924 ,  926 ,  929 ,  932 ,  934 ,  937 ,  940 ,  942 ,  945 ,  948 ,  950 ,  953 ,  956 ,  958 ,  961 ,  963 ,  966 ,  969 ,  971 ,  974 ,  977 ,  980 ,  982 ,  985 ,  988 ,  990 ,  993 ,  996 ,  998 ,  1001 ,  1004 ,  1006 ,  1009 ,  1012 ,  1015 ,  1017 ,  1020 ,  1023 ,  1025 ,  1028 ,  1031 ,  1034 ,  1036 ,  1039 ,  1042 ,  1045 ,  1047 ,  1050 ,  1053 ,  1055 ,  1058 ,  1061 ,  1064 ,  1066 ,  1069 ,  1072 ,  1075 ,  1078 ,  1080 ,  1083 ,  1086 ,  1089 ,  1091 ,  1094 ,  1097 ,  1100 ,  1102 ,  1105 ,  1108 ,  1111 ,  1114 ,  1116 ,  1119 ,  1122 ,  1125 ,  1128 ,  1130 ,  1133 ,  1136 ,  1139 ,  1142 ,  1145 ,  1147 ,  1150 ,  1153 ,  1156 ,  1159 ,  1161 ,  1164 ,  1167 ,  1170 ,  1173 ,  1176 ,  1178 ,  1181 ,  1184 ,  1187 ,  1190 ,  1193 ,  1196 ,  1198 ,  1201 ,  1204 ,  1207 ,  1210 ,  1213 ,  1216 ,  1218 ,  1221 ,  1224 ,  1227 ,  1230 ,  1233 ,  1236 ,  1239 ,  1241 ,  1244 ,  1247 ,  1250 ,  1253 ,  1256 ,  1259 ,  1262 ,  1265 ,  1268 ,  1270 ,  1273 ,  1276 ,  1279 ,  1282 ,  1285 ,  1288 ,  1291 ,  1294 ,  1297 ,  1300 ,  1303 ,  1305 ,  1308 ,  1311 ,  1314 ,  1317 ,  1320 ,  1323 ,  1326 ,  1329 ,  1332 ,  1335 ,  1338 ,  1341 ,  1344 ,  1347 ,  1350 ,  1352 ,  1355 ,  1358 ,  1361 ,  1364 ,  1367 ,  1370 ,  1373 ,  1376 ,  1379 ,  1382 ,  1385 ,  1388 ,  1391 ,  1394 ,  1397 ,  1400 ,  1403 ,  1406 ,  1409 ,  1412 ,  1415 ,  1418 ,  1421 ,  1424 ,  1427 ,  1430 ,  1433 ,  1436 ,  1439 ,  1442 ,  1445 ,  1448 ,  1451 ,  1454 ,  1457 ,  1460 ,  1463 ,  1466 ,  1469 ,  1472 ,  1475 ,  1478 ,  1481 ,  1484 ,  1487 ,  1490 ,  1493 ,  1496 ,  1499 ,  1502 ,  1505 ,  1508 ,  1511 ,  1514 ,  1517 ,  1520 ,  1523 ,  1526 ,  1529 ,  1532 ,  1535 ,  1538 ,  1541 ,  1545 ,  1548 ,  1551 ,  1554 ,  1557 ,  1560 ,  1563 ,  1566 ,  1569 ,  1572 ,  1575 ,  1578 ,  1581 ,  1584 ,  1587 ,  1590 ,  1593 ,  1596 ,  1599 ,  1603 ,  1606 ,  1609 ,  1612 ,  1615 ,  1618 ,  1621 ,  1624 ,  1627 ,  1630 ,  1633 ,  1636 ,  1639 ,  1642 ,  1646 ,  1649 ,  1652 ,  1655 ,  1658 ,  1661 ,  1664 ,  1667 ,  1670 ,  1673 ,  1676 ,  1679 ,  1683 ,  1686 ,  1689 ,  1692 ,  1695 ,  1698 ,  1701 ,  1704 ,  1707 ,  1710 ,  1714 ,  1717 ,  1720 ,  1723 ,  1726 ,  1729 ,  1732 ,  1735 ,  1738 ,  1741 ,  1745 ,  1748 ,  1751 ,  1754 ,  1757 ,  1760 ,  1763 ,  1766 ,  1769 ,  1773 ,  1776 ,  1779 ,  1782 ,  1785 ,  1788 ,  1791 ,  1794 ,  1797 ,  1801 ,  1804 ,  1807 ,  1810 ,  1813 ,  1816 ,  1819 ,  1822 ,  1825 ,  1829 ,  1832 ,  1835 ,  1838 ,  1841 ,  1844 ,  1847 ,  1850 ,  1854 ,  1857 ,  1860 ,  1863 ,  1866 ,  1869 ,  1872 ,  1876 ,  1879 ,  1882 ,  1885 ,  1888 ,  1891 ,  1894 ,  1897 ,  1901 ,  1904 ,  1907 ,  1910 ,  1913 ,  1916 ,  1919 ,  1922 ,  1926 ,  1929 ,  1932 ,  1935 ,  1938 ,  1941 ,  1944 ,  1948 ,  1951 ,  1954 ,  1957 ,  1960 ,  1963 ,  1966 ,  1970 ,  1973 ,  1976 ,  1979 ,  1982 ,  1985 ,  1988 ,  1991 ,  1995 ,  1998 ,  2001 ,  2004 ,  2007 ,  2010 ,  2013 ,  2017 ,  2020 ,  2023 ,  2026 ,  2029 ,  2032 ,  2035 ,  2039 ,  2042 ,  2045 ,
};

#define sine(N) (pgm_read_word_near(sines + ((N) & 4095)))