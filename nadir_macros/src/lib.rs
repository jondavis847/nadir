use proc_macro::TokenStream;
use quote::quote;
use syn::{Data, DeriveInput, LitFloat};

#[proc_macro_derive(Tolerance, attributes(tolerance))]
pub fn derive_tolerance(input: TokenStream) -> TokenStream {
    let input = syn::parse_macro_input!(input as DeriveInput);
    let struct_name = &input.ident;

    let fields = match &input.data {
        Data::Struct(data_struct) => &data_struct.fields,
        _ => panic!("#[derive(Tolerance)] only works on structs"),
    };

    let mut abs_lines = vec![];
    let mut rel_lines = vec![];

    for field in fields.iter() {
        let ident = field
            .ident
            .as_ref()
            .unwrap(); // skip tuple structs

        let mut abs_val = None;
        let mut rel_val = None;

        for attr in &field.attrs {
            if attr
                .path()
                .is_ident("tolerance")
            {
                attr.parse_nested_meta(|meta| {
                    if meta
                        .path
                        .is_ident("abs")
                    {
                        abs_val = Some(
                            meta.value()?
                                .parse::<LitFloat>()?
                                .base10_parse::<f64>()?,
                        );
                    } else if meta
                        .path
                        .is_ident("rel")
                    {
                        rel_val = Some(
                            meta.value()?
                                .parse::<LitFloat>()?
                                .base10_parse::<f64>()?,
                        );
                    }
                    Ok(())
                })
                .unwrap();
            }
        }

        let line = if let (Some(abs), Some(rel)) = (abs_val, rel_val) {
            quote! {
                abs.extend(vec![#abs; self.#ident.len()]);
                rel.extend(vec![#rel; self.#ident.len()]);
            }
        } else {
            quote! {
                let (a, r) = self.#ident.tolerances();
                abs.extend(a);
                rel.extend(r);
            }
        };

        abs_lines.push(line.clone());
        rel_lines.push(line);
    }

    let expanded = quote! {
        impl Tolerances for #struct_name {
            fn tolerances(&self) -> (Vec<f64>, Vec<f64>) {
                let mut abs = vec![];
                let mut rel = vec![];

                #(#abs_lines)*

                (abs, rel)
            }
        }
    };

    TokenStream::from(expanded)
}
