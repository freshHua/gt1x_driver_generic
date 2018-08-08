# gt1x_driver_generic
Touch driver for Goodix GT1-series touch chip

Support QCOM MSM-4.9 kernel,testing for sdm710/sdm845

# Kernel DTS
&tlmm {
		/* add pingrp for touchscreen */
		pmx_ts_int_active {
			ts_int_active: ts_int_active {
				mux {
					pins = "gpio125";
					function = "gpio";
				};

				config {
					pins = "gpio125";
					drive-strength = <8>;
					bias-pull-up;
				};
			};
		};

		pmx_ts_int_suspend {
			ts_int_suspend1: ts_int_suspend1 {
				mux {
					pins = "gpio125";
					function = "gpio";
				};

				config {
					pins = "gpio125";
					drive-strength = <2>;
					bias-pull-down;
				};
			};
		};

		pmx_ts_reset_active {
			ts_reset_active: ts_reset_active {
				mux {
					pins = "gpio99";
					function = "gpio";
				};

				config {
					pins = "gpio99";
					drive-strength = <8>;
					bias-pull-up;
				};
			};
		};

		pmx_ts_reset_suspend {
			ts_reset_suspend1: ts_reset_suspend1 {
				mux {
					pins = "gpio99";
					function = "gpio";
				};

				config {
					pins = "gpio99";
					drive-strength = <2>;
					bias-pull-down;
				};
			};
		};

		pmx_ts_release {
			ts_release: ts_release {
				mux {
					pins = "gpio125", "gpio99";
					function = "gpio";
				};

				config {
					pins = "gpio125", "gpio99";
					drive-strength = <2>;
					bias-pull-down;
				};
			};
		};

		ts_mux {
			ts_active: ts_active {
				mux {
					pins = "gpio99", "gpio125";
					function = "gpio";
				};

				config {
					pins = "gpio99", "gpio125";
					drive-strength = <16>;
					bias-pull-up;
				};
			};

			ts_reset_suspend: ts_reset_suspend {
				mux {
					pins = "gpio99";
					function = "gpio";
				};

				config {
					pins = "gpio99";
					drive-strength = <2>;
					bias-pull-down;
				};
			};

			ts_int_suspend: ts_int_suspend {
				mux {
					pins = "gpio125";
					function = "gpio";
				};

				config {
					pins = "gpio125";
					drive-strength = <2>;
					bias-disable;
				};
			};
		};
};

	&qupv3_se9_i2c {
        status = "ok";
        goodix_ts@5d {
                compatible = "goodix,gt1x";
                reg = <0x5d>;
                interrupt-parent = <&tlmm>;
                interrupts = <125 0x2008>;
                vcc_ana-supply = <&pm660l_l3>;
                vcc_dig-supply = <&pm660_l13>;
                qcom,afe-load = <20000>;
                qcom,afe-vtg-min = <3000000>;
                qcom,afe-vtg-max = <3000000>;
                qcom,dig-load = <40000>;
                qcom,dig-vtg-min = <1800000>;
                qcom,dig-vtg-max = <1800000>;
                qcom,afe-force-power-on;
                qcom,afe-power-on-delay-us = <6>;
                qcom,afe-power-off-delay-us = <6>;
                pinctrl-names = "pmx_ts_active", "pmx_ts_suspend","pmx_ts_release";
                pinctrl-0 = <&ts_int_active &ts_reset_active>;
                pinctrl-1 = <&ts_int_suspend &ts_reset_suspend>;
                pinctrl-2 = <&ts_release>;
                goodix,irq-gpio = <&tlmm 125 0x0>;
                goodix,reset-gpio = <&tlmm 99 0x0>;
        };

};
# ADSP Improvetouch conficted
Improvetouch must be removed from qcom source codes.
